using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using RobotSim.Sensors;
using UnityEditor;
using UnityEngine;

namespace RobotSim.Editor
{
    public static class Mid360DeliveryVerification
    {
        [Serializable] public sealed class Report { public bool passed; public int assertions; public string error; }
        private static Report _report;
        private const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
        private static readonly Type SensorType = typeof(LivoxMid360Sim);
        private static void Check(bool condition, string error)
        {
            _report.assertions++;
            if (!condition) throw new InvalidOperationException(error);
        }
        private static Queue<sensor_msgs.msg.Imu> Imus(LivoxMid360Sim sensor) =>
            (Queue<sensor_msgs.msg.Imu>)SensorType.GetField("_completedImus", Private).GetValue(sensor);
        private static double Stamp(sensor_msgs.msg.Imu imu) => imu.Header.Stamp.Sec + imu.Header.Stamp.Nanosec * 1e-9;
        private static sensor_msgs.msg.Imu Imu(double time) => LivoxMid360Sim.BuildImuMessage(
            new Mid360ImuSampler.Sample { Time = time, Rotation = Quaternion.identity, SpecificForceWorld = Vector3.up * 9.80665f }, "livox_frame");
        private static void EnqueueCloud(LivoxMid360Sim sensor, double start, int count, int offset = 0)
        {
            var frameType = SensorType.GetNestedType("ScanFrame", BindingFlags.NonPublic);
            var frame = Activator.CreateInstance(frameType, true);
            var returns = new Vector2[offset + count];
            for (int i = 0; i < count; i++) returns[offset + i] = i % 7 == 0 ? Vector2.zero : new Vector2(1 + i * 0.0001f, 123);
            frameType.GetField("Returns").SetValue(frame, returns);
            frameType.GetField("ReturnOffset").SetValue(frame, offset);
            frameType.GetField("PointCount").SetValue(frame, count);
            frameType.GetField("StartIndex").SetValue(frame, 0);
            frameType.GetField("Period").SetValue(frame, count / (double)LivoxMid360Sim.PointRate);
            frameType.GetField("EndTime").SetValue(frame, start + count / (double)LivoxMid360Sim.PointRate);
            object queue = SensorType.GetField("_completedScans", Private).GetValue(sensor);
            queue.GetType().GetMethod("Enqueue").Invoke(queue, new[] { frame });
        }
        private static void Drain(LivoxMid360Sim sensor, double horizon, Action<sensor_msgs.msg.Imu> imu,
            Action<sensor_msgs.msg.PointCloud2> cloud) => SensorType.GetMethod("PublishReadyMeasurements", Private)
                .Invoke(sensor, new object[] { horizon, imu, cloud });

        private static void VerifyBacklog(float scale)
        {
            var obj = new GameObject("MID360 delayed delivery fixture");
            obj.SetActive(false); // No real ROS publisher or acquisition loop.
            try
            {
                var sensor = obj.AddComponent<LivoxMid360Sim>();
                SensorType.GetField("_rayDirLivoxRos", Private).SetValue(sensor, new[] { Vector3.right, Vector3.up, Vector3.forward, Vector3.left });
                var imus = Imus(sensor);
                var originalImus = new List<sensor_msgs.msg.Imu>();
                for (int i = 1; i <= 90; i++) { var imu = Imu(10 + i * 0.005); imus.Enqueue(imu); originalImus.Add(imu); }
                // Reproduce six completed 20 Hz scans stalled behind the publisher.
                for (int i = 0; i < 6; i++) EnqueueCloud(sensor, 10 + i * 0.05, 10000, i == 0 ? 3 : 0);
                imus.Enqueue(Imu(10.455));
                EnqueueCloud(sensor, 10.45, 10000);
                Time.timeScale = scale;
                int imuCount = 0, cloudCount = 0;
                double latestImu = 0;
                Action<sensor_msgs.msg.Imu> sendImu = message =>
                {
                    Check(ReferenceEquals(message, originalImus[imuCount]), "Queued IMU measurement was replaced or reordered");
                    Check(Math.Abs(Stamp(message) - (10 + (imuCount + 1) * 0.005)) < 1e-9, "Backlog changed an IMU acquisition timestamp");
                    latestImu = Stamp(message); imuCount++;
                };
                Action<sensor_msgs.msg.PointCloud2> sendCloud = cloud =>
                {
                    double start = 10 + cloudCount * 0.05;
                    Check(cloud.Width == 10000 && cloud.Point_step == 26 && cloud.Data.Length == 260000, "Draining changed official raw cloud layout/rate");
                    Check(Math.Abs(cloud.Header.Stamp.Sec + cloud.Header.Stamp.Nanosec * 1e-9 - start) < 1e-9, "Draining retimestamped a cloud to delivery time");
                    Check(latestImu >= start + 0.05 - 1e-9, "Available IMU coverage was sent after the cloud");
                    for (int i = 0; i < 10000; i++)
                    {
                        Check(Math.Abs(BitConverter.ToDouble(cloud.Data, i * 26 + 18) - (start * 1e9 + i * 5000)) < 0.01,
                            "Catch-up lost, duplicated or retimestamped a LiDAR sample");
                        Check(cloud.Data[i * 26 + 17] == i % 4, "Catch-up changed line order");
                    }
                    Check(BitConverter.ToSingle(cloud.Data, 0) == 0 && BitConverter.ToSingle(cloud.Data, 26 + 12) == 123,
                        "Missing returns or intensity/legacy batch offset changed");
                    cloudCount++;
                };
                // No wall-clock advance: all six old clouds and all due IMUs must
                // leave in one call, including timeScale=0 after acquisition.
                Drain(sensor, 10.45, sendImu, sendCloud);
                Check(imuCount == 90 && cloudCount == 6, "A second delivery clock retained the old backlog");
                Check(sensor.PendingImuCount == 1 && sensor.PendingScanCount == 1, "Future measurements crossed the clock horizon");
                Check(sensor.PublishedImuCount == 90 && sensor.PublishedScanCount == 6, "Delivery counters disagree with actual sends");
                Drain(sensor, 10.45, _ => throw new Exception("Duplicate IMU"), _ => throw new Exception("Duplicate cloud"));
                Check(Math.Abs(sensor.LastPublishedImuTimeSec - 10.45) < 1e-9 &&
                    Math.Abs(sensor.LastPublishedScanEndTimeSec - 10.299995) < 1e-9, "Delivery telemetry uses the wrong measurement time");
                int resumed = 0;
                Drain(sensor, 10.5, message => { Check(Math.Abs(Stamp(message) - 10.455) < 1e-9, "Resume changed IMU time"); resumed++; },
                    cloud => { Check(Math.Abs(BitConverter.ToDouble(cloud.Data, 18) - 10.45e9) < 0.01, "Resume changed cloud time"); resumed++; });
                Check(resumed == 2 && sensor.PendingImuCount == 0 && sensor.PendingScanCount == 0, "Resume did not drain exactly the remaining measurements");
            }
            finally { UnityEngine.Object.DestroyImmediate(obj); }
        }

        private static void VerifyArrivingCoverageAndRates()
        {
            var obj = new GameObject("MID360 concurrent delivery fixture");
            obj.SetActive(false);
            try
            {
                var sensor = obj.AddComponent<LivoxMid360Sim>();
                SensorType.GetField("_rayDirLivoxRos", Private).SetValue(sensor, new[] { Vector3.right, Vector3.up, Vector3.forward, Vector3.left });
                double time = 20, fraction = 0;
                foreach (float hz in new[] { 10f, 20f, 33f, 100f })
                {
                    int count = LivoxMid360Sim.CalculateScanPointCount(hz, ref fraction);
                    EnqueueCloud(sensor, time, count);
                    time += count / (double)LivoxMid360Sim.PointRate;
                }
                var queue = Imus(sensor);
                queue.Enqueue(Imu(20.005));
                // New IMU coverage becomes available while an earlier sample is
                // being sent, after the publisher's initial queue count snapshot.
                double latest = 0;
                Drain(sensor, time, message =>
                {
                    latest = Stamp(message);
                    if (latest < 20.01) queue.Enqueue(Imu(time));
                }, cloud =>
                {
                    double tail = BitConverter.ToDouble(cloud.Data, cloud.Data.Length - 8) * 1e-9;
                    Check(latest >= tail, "IMU arriving during drain was not sent before the next cloud");
                });
                Check(sensor.PublishedScanCount == 4 && sensor.PublishedImuCount == 2 && sensor.PendingScanCount == 0,
                    "Rate changes or concurrent arrivals stranded completed data");
            }
            finally { UnityEngine.Object.DestroyImmediate(obj); }
        }

        [MenuItem("RobotSim/Verify MID360 Delivery")]
        public static void RunMenu() => Debug.Log(Run());
        public static string Run()
        {
            _report = new Report();
            float previousScale = Time.timeScale;
            try
            {
                foreach (float scale in new[] { 0f, 0.25f, 1f, 2f }) VerifyBacklog(scale);
                VerifyArrivingCoverageAndRates();
                _report.passed = true;
            }
            catch (Exception ex) { _report.error = ex.ToString(); }
            finally { Time.timeScale = previousScale; }
            string json = JsonUtility.ToJson(_report, true);
            Directory.CreateDirectory("Temp");
            File.WriteAllText("Temp/Mid360DeliveryVerification.json", json);
            return json;
        }
    }
}
