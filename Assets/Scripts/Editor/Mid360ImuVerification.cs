using System;
using System.Collections.Generic;
using System.IO;
using RobotSim.Sensors;
using UnityEditor;
using UnityEngine;

namespace RobotSim.Editor
{
    public static class Mid360ImuVerification
    {
        [Serializable] public sealed class Report { public bool passed; public int assertions; public string error; }
        private static Report _report;
        private static void Check(bool value, string error) { _report.assertions++; if (!value) throw new InvalidOperationException(error); }
        private static Mid360ImuSampler.State State(double time, Vector3 velocity, Vector3 omega, Quaternion rotation) =>
            new Mid360ImuSampler.State { Time = time, VelocityWorld = velocity, AngularVelocityWorld = omega, Rotation = rotation };
        [MenuItem("RobotSim/Verify MID360 IMU Timing")]
        public static void RunMenu() => Debug.Log(Run());
        private static void VerifyPhysicsBridge()
        {
            var obj = new GameObject("MID360 IMU physics timing fixture");
            try
            {
                var body = obj.AddComponent<Rigidbody>();
                body.useGravity = false;
                body.centerOfMass = new Vector3(0.1f, 0, 0.2f);
                var sensor = obj.AddComponent<LivoxMid360Sim>();
                var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
                var type = typeof(LivoxMid360Sim);
                type.GetField("_baseBody", flags).SetValue(sensor, body);
                type.GetField("baseFrame", flags).SetValue(sensor, obj.transform);
                var capture = type.GetMethod("CaptureImuInterval", flags);
                body.linearVelocity = Vector3.zero; body.angularVelocity = Vector3.zero;
                capture.Invoke(sensor, new object[] { 10.0 });
                body.linearVelocity = Vector3.forward * 0.02f; body.angularVelocity = Vector3.up * 2;
                capture.Invoke(sensor, new object[] { 10.01 });
                var queue = (Queue<sensor_msgs.msg.Imu>)type.GetField("_completedImus", flags).GetValue(sensor);
                var messages = queue.ToArray();
                Check(messages.Length == 2 && sensor.GeneratedImuCount == 2, "Physics bridge did not queue two immutable measurements");
                Check(messages[0].Header.Stamp.Nanosec == 5000000 && messages[1].Header.Stamp.Nanosec == 10000000,
                    "Physics bridge timestamps do not refer to the measured interval");
                Check(Math.Abs(messages[0].Angular_velocity.Z - 0.9629030) < 1e-6 &&
                    Math.Abs(messages[1].Angular_velocity.Z - 1.9258060) < 1e-6,
                    "Physics bridge used latest yaw rate instead of interpolated rate or wrong handedness");
                double preserved = messages[0].Linear_acceleration.X;
                body.linearVelocity = Vector3.forward * 10; body.angularVelocity = Vector3.up * 20;
                capture.Invoke(sensor, new object[] { 10.02 });
                Check(messages[0].Linear_acceleration.X == preserved && messages[0].Header.Stamp.Nanosec == 5000000,
                    "Later Rigidbody state rewrote a previously acquired IMU message");
                Check(messages[0].Linear_acceleration.X != messages[1].Linear_acceleration.X,
                    "IMU lever arm acceleration was duplicated at distinct sample times");
            }
            finally { UnityEngine.Object.DestroyImmediate(obj); }
        }

        public static string Run()
        {
            _report = new Report();
            try
            {
                var samples = new List<Mid360ImuSampler.Sample>();
                var sampler = new Mid360ImuSampler();
                sampler.Capture(State(10, Vector3.zero, Vector3.zero, Quaternion.identity), 200, Vector3.zero, Physics.gravity, samples.Add);
                sampler.Capture(State(10.01, Vector3.forward * 0.01f, Vector3.up, Quaternion.Euler(0, 0.2864789f, 0)), 200, Vector3.zero, Physics.gravity, samples.Add);
                Check(samples.Count == 2, "A 10 ms physics interval must contain two 200 Hz samples");
                Check(Math.Abs(samples[0].Time - 10.005) < 1e-12 && Math.Abs(samples[1].Time - 10.01) < 1e-12, "IMU sample epochs are not 5 ms apart");
                Check(Mathf.Abs(samples[0].AngularVelocityWorld.y - 0.5f) < 1e-6f && samples[1].AngularVelocityWorld.y == 1,
                    "Intermediate IMU values were replaced by the latest state");
                Check((samples[0].SpecificForceWorld - (Vector3.forward - Physics.gravity)).magnitude < 1e-5f, "Acceleration was not derived from the matching physics interval");
                Check(Quaternion.Angle(samples[0].Rotation, Quaternion.Euler(0, 0.14323945f, 0)) < 0.01f, "Sample orientation was not interpolated at its timestamp");
                var frozen = LivoxMid360Sim.BuildImuMessage(samples[0], "livox_frame");
                double expectedGyro = frozen.Angular_velocity.Z;
                sampler.Capture(State(10.02, Vector3.forward * 0.04f, Vector3.up * 3, Quaternion.Euler(0, 1, 0)), 200, Vector3.zero, Physics.gravity, samples.Add);
                Check(frozen.Header.Stamp.Sec == 10 && frozen.Header.Stamp.Nanosec == 5000000 && frozen.Angular_velocity.Z == expectedGyro,
                    "Later physics states mutated an earlier queued measurement");
                Check(frozen.Header.Frame_id == "livox_frame" && frozen.Orientation_covariance[0] == -1, "IMU frame/unknown orientation convention changed");

                // Clock continuity and exact rate, independent of render/executor batching.
                foreach (int hz in new[] { 50, 100, 200, 500, 333 })
                {
                    samples.Clear(); sampler = new Mid360ImuSampler();
                    sampler.Capture(State(10, Vector3.zero, Vector3.zero, Quaternion.identity), hz, Vector3.zero, Vector3.zero, samples.Add);
                    double time = 10;
                    for (int i = 0; i < 100; i++)
                    {
                        time = 10 + (i + 1) * 0.01;
                        sampler.Capture(State(time, Vector3.forward * (float)(time - 10), Vector3.zero, Quaternion.identity), hz, Vector3.zero, Vector3.zero, samples.Add);
                    }
                    Check(samples.Count == hz, "Lost or duplicated IMU samples at " + hz + " Hz");
                    for (int i = 0; i < samples.Count; i++)
                    {
                        var message = LivoxMid360Sim.BuildImuMessage(samples[i], "livox_frame");
                        double stamp = message.Header.Stamp.Sec + message.Header.Stamp.Nanosec * 1e-9;
                        Check(Math.Abs(stamp - (10 + (i + 1.0) / hz)) <= 0.51e-9, "ROS IMU timestamp drifted or used wall time");
                        Check(Mathf.Abs(samples[i].SpecificForceWorld.z - 1) < 1e-4f, "IMU sample used a different interval's acceleration");
                    }
                    Check(samples[samples.Count - 1].Time == 11, "IMU epoch drift across a second boundary");
                    int count = samples.Count;
                    sampler.Capture(State(time, Vector3.zero, Vector3.zero, Quaternion.identity), hz, Vector3.zero, Vector3.zero, samples.Add);
                    Check(samples.Count == count, "Paused physics generated new samples");
                    sampler.Capture(State(time + 0.01, Vector3.zero, Vector3.zero, Quaternion.identity), 500, Vector3.zero, Vector3.zero, samples.Add);
                    Check(samples.Count == count + 5 && samples[count].Time > time, "Rate change reused past epochs");
                }
                // One second of catch-up must interpolate values, not duplicate the last one.
                samples.Clear(); sampler = new Mid360ImuSampler();
                sampler.Capture(State(10, Vector3.zero, Vector3.zero, Quaternion.identity), 200, Vector3.zero, Vector3.zero, samples.Add);
                sampler.Capture(State(11, Vector3.zero, Vector3.up * 2, Quaternion.identity), 200, Vector3.zero, Vector3.zero, samples.Add);
                Check(samples.Count == 200 && Mathf.Abs(samples[0].AngularVelocityWorld.y - 0.01f) < 1e-6f && samples[199].AngularVelocityWorld.y == 2,
                    "Catch-up batch used the final angular velocity for old timestamps");
                LivoxMid360Sim.ToRosTime(10.9999999997, out int sec, out uint ns);
                Check(sec == 11 && ns == 0, "Nanosecond rounding was not normalized across a second");
                var cloud = LivoxMid360Sim.BuildRawPointCloud(new[] { new Vector2(1, 0) }, new[] { Vector3.right }, 0, samples[0].Time, "livox_frame");
                var imu = LivoxMid360Sim.BuildImuMessage(samples[0], "livox_frame");
                Check(cloud.Header.Stamp.Sec == imu.Header.Stamp.Sec && cloud.Header.Stamp.Nanosec == imu.Header.Stamp.Nanosec,
                    "LiDAR and IMU use different timestamp epochs or rounding");

                // Gravity/free fall and the chip's rotational lever arm are evaluated at sample time.
                samples.Clear(); sampler = new Mid360ImuSampler();
                sampler.Capture(State(10, Vector3.zero, Vector3.up * 2, Quaternion.identity), 200, Vector3.right, Vector3.zero, samples.Add);
                sampler.Capture(State(10.01, Vector3.zero, Vector3.up * 2, Quaternion.identity), 200, Vector3.right, Vector3.zero, samples.Add);
                Check((samples[0].SpecificForceWorld - Vector3.left * 4).magnitude < 1e-6f, "Centripetal acceleration at the IMU lever arm is incorrect");
                samples.Clear(); sampler = new Mid360ImuSampler();
                sampler.Capture(State(10, Vector3.zero, Vector3.zero, Quaternion.identity), 200, Vector3.zero, Physics.gravity, samples.Add);
                sampler.Capture(State(10.01, Physics.gravity * 0.01f, Vector3.zero, Quaternion.identity), 200, Vector3.zero, Physics.gravity, samples.Add);
                Check(samples[0].SpecificForceWorld.magnitude < 1e-4f, "Free fall does not produce zero specific force");
                VerifyPhysicsBridge();
                VerifyAccelerationUnits();
                _report.passed = true;
            }
            catch (Exception ex) { _report.error = ex.ToString(); }
            Directory.CreateDirectory("Temp");
            string result = JsonUtility.ToJson(_report, true);
            File.WriteAllText("Temp/Mid360ImuVerification.json", result);
            if (!_report.passed) throw new InvalidOperationException(result);
            return result;
        }

        private static void VerifyAccelerationUnits()
        {
            var sample = new Mid360ImuSampler.Sample
            {
                Time = 12.345, Rotation = Quaternion.identity,
                AngularVelocityWorld = Vector3.up * 2,
                SpecificForceWorld = Vector3.up * 9.80665f
            };
            var imu = LivoxMid360Sim.BuildImuMessage(sample, "livox_frame");
            double magnitude = Math.Sqrt(imu.Linear_acceleration.X * imu.Linear_acceleration.X +
                imu.Linear_acceleration.Y * imu.Linear_acceleration.Y + imu.Linear_acceleration.Z * imu.Linear_acceleration.Z);
            Check(Math.Abs(magnitude - 1) < 1e-6, "Standard gravity must publish 1 g, not 9.80665 m/s²");
            Check(Math.Abs(imu.Angular_velocity.Z - 1.925806) < 1e-6, "Acceleration conversion changed gyro rad/s");
            Check(imu.Header.Stamp.Sec == 12 && imu.Header.Stamp.Nanosec == 345000000,
                "Acceleration conversion changed measurement timestamp");
            sample.SpecificForceWorld = new Vector3(1, 2, 3) * 9.80665f;
            imu = LivoxMid360Sim.BuildImuMessage(sample, "livox_frame");
            Check(Math.Abs(imu.Linear_acceleration.X - 3.4281422) < 1e-6 &&
                Math.Abs(imu.Linear_acceleration.Y - 1.0127720) < 1e-6 &&
                Math.Abs(imu.Linear_acceleration.Z + 1.1055025) < 1e-6,
                "All mounted accelerometer axes must publish signed g values");
            sample.SpecificForceWorld = Vector3.zero;
            imu = LivoxMid360Sim.BuildImuMessage(sample, "livox_frame");
            Check(imu.Linear_acceleration.X == 0 && imu.Linear_acceleration.Y == 0 && imu.Linear_acceleration.Z == 0,
                "Zero specific force must remain zero g");
        }
    }
}
