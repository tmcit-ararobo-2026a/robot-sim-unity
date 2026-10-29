using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using RobotSim.Sensors;
using UnityEditor;
using UnityEngine;

namespace RobotSim.Editor
{
    public static class Mid360MeshVerification
    {
        [Serializable] public sealed class Report
        {
            public bool passed;
            public int assertions;
            public string gpu, error;
            public double submit20000Ms, readback20000Ms;
            public float maximumErrorMeters;
        }
        private static Report _report;
        private static void Check(bool condition, string message)
        {
            _report.assertions++;
            if (!condition) throw new InvalidOperationException(message);
        }
        private static Mid360TemporalScene.TimedRay Ray(Vector3 origin, Vector3 direction, float fraction) =>
            new Mid360TemporalScene.TimedRay
            {
                Origin = new Vector4(origin.x, origin.y, origin.z, fraction),
                Direction = new Vector4(direction.x, direction.y, direction.z, 0)
            };
        private static GameObject Box(string name, Vector3 position, Vector3 scale)
        {
            var obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.name = name; obj.layer = 31;
            obj.transform.position = position; obj.transform.localScale = scale;
            UnityEngine.Object.DestroyImmediate(obj.GetComponent<Collider>());
            return obj;
        }
        [MenuItem("RobotSim/Verify MID360 Mesh BVH")]
        public static void RunMenu() => UnityEngine.Debug.Log(Run());

        public static string Run()
        {
            if (Application.isPlaying) throw new InvalidOperationException("Run outside Play mode");
            _report = new Report { gpu = SystemInfo.graphicsDeviceName };
            var objects = new List<GameObject>();
            var meshes = new List<Mesh>();
            var origin = new Vector3(100, 200, 300);
            try
            {
                using var scene = new Mid360TemporalScene(true, 1 << 31);
                Check(scene.UsingGpu, "Mesh GPU path is required for this regression");
                // Empty scenes must work without unbound buffers or undefined reads.
                scene.Capture();
                scene.Submit(new[] { Ray(origin, Vector3.forward, 0) }, 0.1f, 100, result =>
                    Check(result.Length == 1 && result[0].x < 0, "Empty scene returned a hit"));
                scene.WaitForPending();
                var wall = Box("MID360 analytic wall", origin + Vector3.forward * 10, new Vector3(100, 100, 0.2f));
                objects.Add(wall);
                wall.AddComponent<Mid360Surface>().reflectivity = 123;
                scene.Capture();
                scene.Capture();
                var rays = new Mid360TemporalScene.TimedRay[20000];
                for (int i = 0; i < rays.Length; i++)
                {
                    float fraction = i / 20000f;
                    LivoxMid360Sim.GetTimedRay(origin, Quaternion.identity, origin + Vector3.forward,
                        Quaternion.Euler(0, 60, 0), fraction, Vector3.zero, Vector3.right, out var start, out var direction);
                    rays[i] = Ray(start, direction, fraction);
                }
                var watch = Stopwatch.StartNew();
                scene.Submit(rays, 0.1f, 100, result =>
                {
                    _report.readback20000Ms = watch.Elapsed.TotalMilliseconds;
                    for (int i = 0; i < result.Length; i++)
                    {
                        float fraction = i / 20000f;
                        float expected = (9.9f - fraction) / Mathf.Cos(fraction * Mathf.PI / 3);
                        float error = Mathf.Abs(result[i].x - expected);
                        _report.maximumErrorMeters = Mathf.Max(_report.maximumErrorMeters, error);
                        Check(error < 0.002f && result[i].y == 123, "Timed GPU ray failed analytic wall range/reflectivity at " + i + ": " + result[i] + ", expected " + expected);
                    }
                });
                _report.submit20000Ms = watch.Elapsed.TotalMilliseconds;
                scene.WaitForPending();
                // Different snapshots remain independent while multiple dispatches are in flight.
                int callbacks = 0;
                for (int step = 1; step <= 4; step++)
                {
                    wall.transform.position = origin + Vector3.forward * (10 + step);
                    wall.transform.rotation = Quaternion.Euler(0, step * 5, 0);
                    scene.Capture();
                    var moving = new[] { Ray(origin, Vector3.forward, 0), Ray(origin, Vector3.forward, 0.37f), Ray(origin, Vector3.forward, 1) };
                    var expected = new Vector2[moving.Length];
                    for (int i = 0; i < expected.Length; i++) expected[i] = scene.Sample(origin, Vector3.forward, moving[i].Origin.w, 0.1f, 100);
                    int order = step;
                    scene.Submit(moving, 0.1f, 100, result =>
                    {
                        Check(++callbacks == order, "GPU mesh readbacks arrived out of order");
                        for (int i = 0; i < result.Length; i++)
                            Check(Mathf.Abs(result[i].x - expected[i].x) < 0.002f, "GPU moving instance differs from CPU mesh reference");
                    });
                }
                scene.WaitForPending();
                Check(callbacks == 4, "Mesh acquisition lost an in-flight interval");
                UnityEngine.Object.DestroyImmediate(wall);
                objects.Clear();

                // A frame has a real hole, despite a solid box collider covering it.
                var frame = new GameObject("MID360 frame with hole") { layer = 31 };
                objects.Add(frame);
                frame.transform.position = origin + Vector3.forward * 5;
                Mesh mesh = FrameMesh(); meshes.Add(mesh);
                frame.AddComponent<MeshFilter>().sharedMesh = mesh;
                frame.AddComponent<MeshRenderer>();
                frame.AddComponent<BoxCollider>().size = new Vector3(4, 4, 1);
                var back = Box("MID360 back wall", origin + Vector3.forward * 10, new Vector3(100, 100, 0.2f));
                objects.Add(back);
                var thin = Box("MID360 thin render-only part", origin + new Vector3(0.123f, 0, 3), new Vector3(0.001f, 0.01f, 0.001f));
                objects.Add(thin);
                scene.Capture(); scene.Capture();
                var detail = new[]
                {
                    Ray(origin, Vector3.forward, 0),
                    Ray(origin, new Vector3(1.5f, 0, 5).normalized, 0),
                    Ray(origin, new Vector3(0.123f, 0, 3).normalized, 0)
                };
                scene.Submit(detail, 0.1f, 100, result =>
                {
                    Check(Mathf.Abs(result[0].x - 9.9f) < 0.002f, "Solid collider incorrectly filled mesh hole");
                    Check(Mathf.Abs(result[1].x - Mathf.Sqrt(25 + 2.25f)) < 0.002f, "Frame triangle surface missing");
                    Check(result[2].x > 2.99f && result[2].x < 3.01f, "Sub-pixel render-only triangle was missed");
                });
                scene.WaitForPending();
                // Non-uniform scale and negative scale retain world metre distances.
                frame.transform.rotation = Quaternion.Euler(13, 29, 7);
                frame.transform.localScale = new Vector3(-2, 0.5f, 3);
                scene.Capture();
                var scaledRays = new Mid360TemporalScene.TimedRay[97];
                var scaledExpected = new Vector2[97];
                for (int i = 0; i < 97; i++)
                {
                    Vector3 direction = new Vector3((i % 11 - 5) * 0.05f, (i % 7 - 3) * 0.05f, 1).normalized;
                    float fraction = i / 96f;
                    scaledRays[i] = Ray(origin, direction, fraction);
                    scaledExpected[i] = scene.Sample(origin, direction, fraction, 0.1f, 100);
                }
                scene.Submit(scaledRays, 0.1f, 100, result =>
                {
                    for (int i = 0; i < result.Length; i++) Check(Mathf.Abs(result[i].x - scaledExpected[i].x) < 0.003f,
                        "Scaled GPU mesh intersection differs from CPU reference");
                });
                scene.WaitForPending();
                frame.GetComponent<MeshRenderer>().enabled = false;
                thin.GetComponent<MeshRenderer>().enabled = false;
                back.GetComponent<MeshRenderer>().enabled = false;
                scene.Capture();
                scene.Submit(new[] { Ray(origin, Vector3.forward, 1) }, 0.1f, 100,
                    result => Check(result[0].x < 0, "Disabled renderers still produced mesh returns"));
                scene.WaitForPending();
                VerifySkin(scene, origin, objects, meshes);
                Check(scene.UsingGpu, "Verification silently fell back from GPU");
                _report.passed = true;
            }
            catch (Exception ex) { _report.error = ex.ToString(); }
            finally
            {
                foreach (var obj in objects) if (obj != null) UnityEngine.Object.DestroyImmediate(obj);
                foreach (var mesh in meshes) if (mesh != null) UnityEngine.Object.DestroyImmediate(mesh);
            }
            Directory.CreateDirectory("Temp");
            string json = JsonUtility.ToJson(_report, true);
            File.WriteAllText("Temp/Mid360MeshVerification.json", json);
            if (!_report.passed) throw new InvalidOperationException(json);
            return json;
        }

        private static Mesh FrameMesh()
        {
            var vertices = new List<Vector3>(); var indices = new List<int>();
            AddQuad(vertices, indices, -2, -2, -1, 2);
            AddQuad(vertices, indices, 1, -2, 2, 2);
            AddQuad(vertices, indices, -1, -2, 1, -1);
            AddQuad(vertices, indices, -1, 1, 1, 2);
            var mesh = new Mesh { name = "MID360 frame triangles" };
            mesh.SetVertices(vertices); mesh.SetTriangles(indices, 0); mesh.RecalculateBounds();
            return mesh;
        }
        private static void AddQuad(List<Vector3> vertices, List<int> indices, float x0, float y0, float x1, float y1)
        {
            int b = vertices.Count;
            vertices.Add(new Vector3(x0, y0, 0)); vertices.Add(new Vector3(x1, y0, 0));
            vertices.Add(new Vector3(x1, y1, 0)); vertices.Add(new Vector3(x0, y1, 0));
            indices.AddRange(new[] { b, b + 1, b + 2, b, b + 2, b + 3 });
        }
        private static void VerifySkin(Mid360TemporalScene scene, Vector3 origin, List<GameObject> objects, List<Mesh> meshes)
        {
            var obj = new GameObject("MID360 deforming mesh") { layer = 31 };
            objects.Add(obj); obj.transform.position = origin;
            var bone = new GameObject("bone"); bone.transform.SetParent(obj.transform, false);
            var mesh = new Mesh { name = "MID360 skinned test" }; meshes.Add(mesh);
            mesh.vertices = new[] { new Vector3(-1,-1,5), new Vector3(1,-1,5), new Vector3(0,1,5) };
            mesh.triangles = new[] { 0, 1, 2 };
            mesh.bindposes = new[] { Matrix4x4.identity };
            var weight = new BoneWeight { boneIndex0 = 0, weight0 = 1 };
            mesh.boneWeights = new[] { weight, weight, weight };
            mesh.RecalculateBounds();
            var skin = obj.AddComponent<SkinnedMeshRenderer>();
            skin.sharedMesh = mesh; skin.bones = new[] { bone.transform }; skin.rootBone = bone.transform;
            scene.Capture();
            bone.transform.localPosition = Vector3.forward * 2;
            scene.Capture();
            scene.Submit(new[] { Ray(origin, Vector3.forward, 0), Ray(origin, Vector3.forward, 0.5f), Ray(origin, Vector3.forward, 1) },
                0.1f, 100, result =>
                {
                    for (int i = 0; i < 3; i++) Check(Mathf.Abs(result[i].x - (5 + i)) < 0.002f,
                        "Skinned vertices were not interpolated at the ray time");
                });
            // Rebuild geometry while the prior readback remains in flight.
            bone.transform.localPosition = Vector3.forward * 3;
            scene.Capture();
            scene.Submit(new[] { Ray(origin, Vector3.forward, 0.5f) }, 0.1f, 100,
                result => Check(Mathf.Abs(result[0].x - 7.5f) < 0.002f, "New skinned upload corrupted an earlier interval"));
            scene.WaitForPending();
        }
    }
}
