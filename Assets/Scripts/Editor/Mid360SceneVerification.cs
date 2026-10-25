using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using RobotSim.Sensors;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace RobotSim.Editor
{
    /// <summary>GPU regression checks against analytic/PhysX distances. No saved scene changes.</summary>
    public static class Mid360SceneVerification
    {
        [Serializable]
        public sealed class Report
        {
            public bool passed;
            public string error;
            public string gpu;
            public string graphicsApi;
            public int assertions;
            public float maxRangeErrorMeters;
            public double cpu20000RaysMs;
            public double gpu20000RaysSubmitMs;
            public double gpu20000RaysLatencyMs;
        }

        private static Mid360SceneSampler _gpu;
        private static Mid360SceneSampler _cpu;
        private static GameObject _fixture;
        private static readonly List<GameObject> Targets = new List<GameObject>();
        private static readonly Vector3 Origin = new Vector3(100, 200, 300);
        private static Vector3[] _rays;
        private static Vector2[] _baseline;
        private static Report _report;
        private static int _phase;
        private static double _deadline;
        private static Action _next;
        private const int TestMask = 1 << 31;
        public static bool Running => _fixture != null;
        public const string ReportPath = "Temp/Mid360Verification.json";

        [MenuItem("RobotSim/Verify MID360 Scene Capture")]
        public static void Start()
        {
            if (Running) throw new InvalidOperationException("Verification already running");
            if (Application.isPlaying) throw new InvalidOperationException("Run verification outside Play mode");
            _report = new Report { gpu = SystemInfo.graphicsDeviceName, graphicsApi = SystemInfo.graphicsDeviceType.ToString() };
            Directory.CreateDirectory("Temp");
            File.Delete(ReportPath);
            try
            {
                VerifyRawOutput();
                _fixture = new GameObject("MID360 Verification") { hideFlags = HideFlags.HideAndDontSave };
                var rays = new List<Vector3>();
                Vector3[] axes = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };
                foreach (var axis in axes)
                {
                    rays.Add(axis);
                    rays.Add((axis + new Vector3(0.17f, -0.13f, 0.21f)).normalized);
                }
                rays.Add(new Vector3(1, 1, 0).normalized);
                rays.Add(new Vector3(-1, 0, 1).normalized);
                rays.Add(new Vector3(0, -1, -1).normalized);
                _rays = rays.ToArray();
                foreach (var ray in _rays) Targets.Add(Cube(Origin + ray * 3f, Vector3.one * 0.3f));
                // An own-robot surface must remain a real first return, with the
                // obstacle behind it occluded. Keep it outside the minimum range.
                var self = Cube(Origin + _rays[6] * 1f, Vector3.one * 0.3f);
                self.transform.SetParent(_fixture.transform, true);
                var surface = self.AddComponent<Mid360Surface>();
                surface.reflectivity = 123;
                surface.Apply();
                _gpu = new Mid360SceneSampler(true, 512);
                _cpu = new Mid360SceneSampler(false, 512);
                Check(_gpu.UsingGpu, "GPU path unavailable; CPU fallback is not a GPU test pass");
                _phase = 0;
                _deadline = EditorApplication.timeSinceStartup + 30;
                EditorApplication.update += Tick;
                _next = BeginPhase;
            }
            catch (Exception ex) { Finish(ex); }
        }

        private static void VerifyRawOutput()
        {
            Check(LivoxMid360Sim.MinRange == 0.1f && LivoxMid360Sim.MaxRange == 100f, "MID-360S physical range limits changed");
            Check(LivoxMid360Sim.PointRate == 200000 && LivoxMid360Sim.PointsPerScan == 20000, "MID-360S point rate changed");
            Check(LivoxMid360Sim.ImuPeriod == 0.005, "MID-360S IMU must use 200 Hz");
            var returns = new Vector2[20000];
            for (int i = 0; i < returns.Length; i++) returns[i] = new Vector2(1.09f, 75);
            returns[0] = new Vector2(0.15f, 123); // Own surface in the former 0.2 m blind zone.
            returns[1] = new Vector2(-1, 0); // Missing return must occupy its sample slot.
            var cloud = LivoxMid360Sim.BuildRawPointCloud(returns, new[] { new Vector3(0, 0, -1) }, 0, 10, "livox_frame");
            Check(cloud.Width == 20000 && cloud.Data.Length == 500000, "Raw floor/self points were downsampled or capped");
            Check(Mathf.Abs(BitConverter.ToSingle(cloud.Data, 8) + 0.15f) < 1e-6f, "Near own-surface return removed");
            Check(BitConverter.ToSingle(cloud.Data, 12) == 123, "Raw reflectivity changed");
            Check(BitConverter.ToSingle(cloud.Data, 25) == 0 && BitConverter.ToSingle(cloud.Data, 29) == 0 &&
                BitConverter.ToSingle(cloud.Data, 33) == 0, "Missing return is not (0,0,0)");
            Check(Math.Abs(BitConverter.ToDouble(cloud.Data, 41) - BitConverter.ToDouble(cloud.Data, 16) - 5000) < 0.01,
                "Raw per-point timestamps are not spaced by 5 microseconds");
            Check(Mathf.Abs(BitConverter.ToSingle(cloud.Data, 499983) + 1.09f) < 1e-6f, "Tail of raw floor scan lost");
            Check(cloud.Fields.Length == 6 && cloud.Fields[5].Name == "tag" && cloud.Data[24] == 0, "Normal return tag missing");
        }

        private static GameObject Cube(Vector3 position, Vector3 scale)
        {
            var obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.name = "MID360 Test Target";
            obj.hideFlags = HideFlags.HideAndDontSave;
            obj.layer = 31;
            obj.transform.position = position;
            obj.transform.localScale = scale;
            return obj;
        }

        private static void Tick()
        {
            try
            {
                if (EditorApplication.timeSinceStartup > _deadline) throw new TimeoutException("GPU verification timed out");
                var next = _next;
                _next = null;
                next?.Invoke();
                EditorApplication.QueuePlayerLoopUpdate();
            }
            catch (Exception ex) { Finish(ex); }
        }

        private static void BeginPhase()
        {
            if (_phase == 1)
            {
                UnityEngine.Object.DestroyImmediate(Targets[0].GetComponent<Collider>());
                Targets[1].transform.position += _rays[1];
                Targets[2].GetComponent<Renderer>().enabled = false;
                Targets[3].layer = 30;
                Targets.Add(Cube(Origin + _rays[4] * 1.5f, Vector3.one * 0.2f));
            }
            if (_phase == 4) { Benchmark(); return; }
            float min = _phase == 3 ? 2f : 0.2f;
            float max = _phase == 2 ? 2f : 10f;
            Vector2[] expected = null;
            _cpu.TryScan(Origin, _rays, min, max, TestMask, result => expected = result);
            if (_phase == 0) _baseline = expected;
            bool submitted = _gpu.TryScan(Origin, _rays, min, max, TestMask, result =>
            {
                try
                {
                    Check(_gpu.UsingGpu, "GPU unexpectedly fell back to CPU");
                    for (int i = 0; i < _rays.Length; i++)
                    {
                        float wanted = expected[i].x;
                        if (_phase == 1 || _phase == 3)
                        {
                            if (i == 0) wanted = _baseline[0].x; // Render-only mesh remains detectable.
                            if (i == 2) wanted = -1; // Disabled renderer is invisible to GPU.
                        }
                        if (_phase == 2 && i == 0) wanted = -1;
                        Check((wanted > 0) == (result[i].x > 0), $"Phase {_phase}, ray {i}: expected {wanted}, got {result[i]}");
                        if (wanted <= 0) continue;
                        float error = Mathf.Abs(wanted - result[i].x);
                        _report.maxRangeErrorMeters = Mathf.Max(_report.maxRangeErrorMeters, error);
                        Check(error < 0.04f, $"Phase {_phase}, ray {i}: range error {error} m");
                    }
                    if (_phase == 0)
                    {
                        Check(result[6].x > 0f && result[6].x < 1.2f, "Own-robot surface was removed");
                        Check(Mathf.Abs(result[6].y - 123) < 0.1f, "Assigned reflectivity was not preserved");
                        Check(!_fixture.GetComponentInChildren<Renderer>().forceRenderingOff, "Renderer state was changed");
                    }
                    _phase++;
                    _next = BeginPhase;
                }
                catch (Exception ex) { Finish(ex); }
            });
            Check(submitted, "Initial GPU request rejected");
            Check(!_gpu.TryScan(Origin, _rays, min, max, TestMask, _ => { }), "In-flight scan must reject a second request");
            // Edit-mode GPU readbacks may not be pumped while the Game view is idle.
            // Only this diagnostic waits; the actual Play-mode path remains asynchronous.
            _gpu.WaitForPendingScan();
        }

        private static void Benchmark()
        {
            var rays = new Vector3[20000];
            for (int i = 0; i < rays.Length; i++) rays[i] = _rays[i % _rays.Length];
            var watch = Stopwatch.StartNew();
            for (int i = 0; i < 5; i++) _cpu.TryScan(Origin, rays, 0.2f, 10, TestMask, _ => { });
            _report.cpu20000RaysMs = watch.Elapsed.TotalMilliseconds / 5;
            watch.Restart();
            _gpu.TryScan(Origin, rays, 0.2f, 10, TestMask, _ =>
            {
                _report.gpu20000RaysLatencyMs = watch.Elapsed.TotalMilliseconds;
                _next = () => Finish(null);
            });
            _report.gpu20000RaysSubmitMs = watch.Elapsed.TotalMilliseconds;
            _gpu.WaitForPendingScan();
        }

        private static void Check(bool condition, string message)
        {
            _report.assertions++;
            if (!condition) throw new InvalidOperationException(message);
        }

        private static void Finish(Exception error)
        {
            EditorApplication.update -= Tick;
            _next = null;
            _gpu?.Dispose();
            _cpu?.Dispose();
            _gpu = null;
            _cpu = null;
            foreach (var target in Targets) if (target != null) UnityEngine.Object.DestroyImmediate(target);
            Targets.Clear();
            if (_fixture != null) UnityEngine.Object.DestroyImmediate(_fixture);
            _fixture = null;
            _report.passed = error == null;
            _report.error = error?.ToString();
            File.WriteAllText(ReportPath, JsonUtility.ToJson(_report, true));
            if (error == null) Debug.Log("[MID360 Verification] PASS: " + ReportPath);
            else Debug.LogError("[MID360 Verification] " + error);
        }
    }
}
