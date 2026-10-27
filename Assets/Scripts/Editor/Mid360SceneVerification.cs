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
        private static int _phaseCompletions;
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
                // Message constructors need ROS2's native-library search paths,
                // including when this check is run before the first Play session.
                var ros2 = UnityEngine.Object.FindFirstObjectByType<ROS2.ROS2UnityComponent>();
                if (ros2 == null || !ros2.Ok())
                    throw new InvalidOperationException("MID360 verification requires an initialized ROS2 manager.");
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
            double fractionalPoints = 0;
            Check(LivoxMid360Sim.CalculateScanPointCount(10, ref fractionalPoints) == 20000, "Default scan point count changed");
            Check(LivoxMid360Sim.CalculateScanPointCount(20, ref fractionalPoints) == 10000, "20 Hz must retain 200,000 samples/s");
            Check(LivoxMid360Sim.CalculateScanPointCount(5, ref fractionalPoints) == 40000, "5 Hz must retain all raw samples");
            fractionalPoints = 0;
            int samplesPerSecond = 0;
            for (int i = 0; i < 30; i++) samplesPerSecond += LivoxMid360Sim.CalculateScanPointCount(30, ref fractionalPoints);
            Check(samplesPerSecond == LivoxMid360Sim.PointRate, "Fractional scan sizes lost samples at 30 Hz");
            VerifyInspectorRateLimits();
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
            VerifyBatchedRawOutput();
        }

        private static void VerifyInspectorRateLimits()
        {
            var obj = new GameObject("MID360 Rate Limits") { hideFlags = HideFlags.HideAndDontSave };
            try
            {
                var sensor = obj.AddComponent<LivoxMid360Sim>();
                var serialized = new SerializedObject(sensor);
                var cloudRate = serialized.FindProperty("pointCloudPublishRateHz");
                var imuRate = serialized.FindProperty("imuPublishRateHz");
                cloudRate.floatValue = 101;
                imuRate.floatValue = 501;
                serialized.ApplyModifiedPropertiesWithoutUndo();
                serialized.Update();
                Check(cloudRate.floatValue == 100 && sensor.PointCloudPublishRateHz == 100,
                    "Inspector must limit point-cloud publication to 100 Hz");
                Check(imuRate.floatValue == 500 && sensor.ImuPublishRateHz == 500,
                    "Inspector must limit IMU publication to 500 Hz");
                double fractionalPoints = 0;
                Check(LivoxMid360Sim.CalculateScanPointCount(101, ref fractionalPoints) == 2000,
                    "Runtime point budget exceeded the 100 Hz limit");
            }
            finally { UnityEngine.Object.DestroyImmediate(obj); }
        }

        private static void VerifyBatchedRawOutput()
        {
            // A readback spans multiple ROS windows, including a missing return
            // and a close own-surface return across a scan-pattern wrap.
            var returns = new[]
            {
                new Vector2(1, 10), new Vector2(2, 20), new Vector2(3, 30),
                new Vector2(-1, 0), new Vector2(0.15f, 123),
                new Vector2(6, 60), new Vector2(7, 70), new Vector2(8, 80)
            };
            var pattern = new[] { Vector3.right, Vector3.up, Vector3.forward };
            const double start = 10;
            var whole = LivoxMid360Sim.BuildRawPointCloud(returns, pattern, 2, start, "livox_frame");
            int[] counts = { 3, 5 };
            int offset = 0;
            double previousTail = 0;
            foreach (int count in counts)
            {
                double scanStart = start + (double)offset / LivoxMid360Sim.PointRate;
                var part = LivoxMid360Sim.BuildRawPointCloud(returns, pattern, (2 + offset) % pattern.Length,
                    scanStart, "livox_frame", offset, count);
                Check(part.Width == count && part.Data.Length == count * 25, "Batched cloud lost its window boundaries");
                bool equal = true;
                for (int b = 0; b < part.Data.Length; b++)
                    if (b % 25 < 16 || b % 25 == 24)
                        equal &= part.Data[b] == whole.Data[offset * 25 + b];
                // Equivalent double-precision timestamp expressions can differ
                // by a few millionths of a nanosecond after window splitting.
                for (int p = 0; p < count; p++)
                    equal &= Math.Abs(BitConverter.ToDouble(part.Data, p * 25 + 16) -
                        BitConverter.ToDouble(whole.Data, (offset + p) * 25 + 16)) < 0.01;
                Check(equal, "Batched scan changed ranges, reflectivity, pattern order or per-point timestamps");
                double first = BitConverter.ToDouble(part.Data, 16);
                if (offset > 0) Check(Math.Abs(first - previousTail - 5000) < 0.01, "Batch boundary lost raw samples");
                previousTail = BitConverter.ToDouble(part.Data, (count - 1) * 25 + 16);
                Check(Math.Abs(part.Header.Stamp.Sec + part.Header.Stamp.Nanosec * 1e-9 - scanStart) < 2e-9,
                    "Batched cloud header refers to another scan window");
                offset += count;
            }
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
                _gpu?.PollReadback();
                var next = _next;
                _next = null;
                next?.Invoke();
                EditorApplication.QueuePlayerLoopUpdate();
            }
            catch (Exception ex) { Finish(ex); }
        }

        private static void BeginPhase()
        {
            _phaseCompletions = 0;
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
                    CompletePhaseScan();
                }
                catch (Exception ex) { Finish(ex); }
            });
            Check(submitted, "Initial GPU request rejected");
            float extraRange0 = _phase == 1 || _phase == 3 ? _baseline[0].x : expected[0].x;
            float extraRange1 = expected[1].x;
            float extraRange2 = _phase == 1 || _phase == 3 ? -1 : expected[2].x;
            Check(_gpu.TryScan(Origin, new[] { _rays[0] }, min, max, TestMask, result =>
            {
                Check(_phaseCompletions == 1, "Readbacks were delivered out of acquisition order");
                Check(result.Length == 1, "First in-flight scan reused another scan's buffer");
                Check(Mathf.Abs(result[0].x - extraRange0) < 0.04f, "First in-flight scan returned another scan's distances");
                CompletePhaseScan();
            }), "Second in-flight scan rejected");
            Check(_gpu.TryScan(Origin, new[] { _rays[1], _rays[2] }, min, max, TestMask, result =>
            {
                Check(_phaseCompletions == 2, "Readbacks were delivered out of acquisition order");
                Check(result.Length == 2, "Second in-flight scan reused another scan's buffer");
                Check(Mathf.Abs(result[0].x - extraRange1) < 0.04f && Mathf.Abs(result[1].x - extraRange2) < 0.04f,
                    "Second in-flight scan returned another scan's distances");
                CompletePhaseScan();
            }), "Third in-flight scan rejected");
            Check(!_gpu.TryScan(Origin, _rays, min, max, TestMask, _ => { }), "A full readback queue must apply backpressure");
            // Edit mode may park the render device between updates even when its
            // request status is polled. Force completion only in this diagnostic;
            // Play-mode acquisition never waits and is checked with ROS receivers.
            _gpu.WaitForPendingScan();
        }

        private static void CompletePhaseScan()
        {
            if (++_phaseCompletions < Mid360SceneSampler.MaxPendingReadbacks) return;
            _phase++;
            _next = BeginPhase;
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
