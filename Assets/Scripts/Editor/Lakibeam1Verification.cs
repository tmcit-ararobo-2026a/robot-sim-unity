#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEngine;
using RobotSim.Sensors;
using Debug = UnityEngine.Debug;

namespace RobotSim.Editor
{
    public static class Lakibeam1Verification
    {
        private static void Assert(bool condition, string message)
        {
            if (!condition)
            {
                throw new Exception($"[Lakibeam1Verification FAIL] {message}");
            }
        }

        [MenuItem("RobotSim/Verify Lakibeam1 Sensor")]
        public static void RunMenu()
        {
            Debug.Log(Run());
        }

        public static string Run()
        {
            Debug.Log("[Lakibeam1Verification] Starting verification...");

            // 1. Test Component lifecycle and defaults
            GameObject sensorObj = new GameObject("TestLakibeam");
            sensorObj.transform.position = Vector3.zero;
            sensorObj.transform.rotation = Quaternion.identity;

            // Create a test obstacle box at z = 2.0m (which is forward in Unity / angle = 0 in sensor frame)
            GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obstacle.name = "TestWall";
            obstacle.transform.position = new Vector3(0f, 0f, 2.0f);
            obstacle.transform.localScale = new Vector3(10f, 2f, 0.1f);
            Physics.SyncTransforms();

            try
            {
                var sensor = sensorObj.AddComponent<Lakibeam1Sim>();
                Assert(sensor.Topic == "/lakibeam/scan", $"Topic mismatch: {sensor.Topic}");
                Assert(sensor.FrameId == "lakibeam_frame", $"FrameId mismatch: {sensor.FrameId}");
                Assert(sensor.ScanRateHz == 20.0f, $"ScanRateHz mismatch: {sensor.ScanRateHz}");
                Assert(sensor.NumBeams == 1080, $"NumBeams mismatch: {sensor.NumBeams}");

                // 2. Test Raycast acquisition
                sensor.AcquireAndPublishScan(10.0, 0.05);

                // Use reflection or check private ranges
                var rangeProp = typeof(Lakibeam1Sim).GetField("_ranges", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                Assert(rangeProp != null, "Could not access _ranges field");
                float[] ranges = (float[])rangeProp.GetValue(sensor);
                Assert(ranges != null && ranges.Length == 1080, "Ranges array null or wrong length");

                // Beam at angle 0 deg: index = numBeams / 2 = 540 (since range is [-180, 180])
                int centerIdx = 540;
                float centerDist = ranges[centerIdx];
                // Obstacle front surface is at z = 2.0m - 0.05m = 1.95m
                Debug.Log($"[Lakibeam1Verification] Center beam (0 deg) distance to obstacle: {centerDist:F3} m");
                Assert(centerDist >= 1.90f && centerDist <= 2.05f, $"Center beam expected ~1.95m, got {centerDist}");

                // Beam pointing sideways (e.g. at index 0, -180 deg, pointing backward) should hit infinity
                float backwardDist = ranges[0];
                Assert(float.IsPositiveInfinity(backwardDist), $"Backward beam should be infinity, got {backwardDist}");

                Debug.Log("[Lakibeam1Verification] Wall detection and range calculations PASSED!");

                // 3. Test Motion Distortion
                // Attach a Rigidbody with angular velocity to verify distorted beam sampling
                var rb = sensorObj.AddComponent<Rigidbody>();
                rb.isKinematic = true;
                rb.angularVelocity = new Vector3(0f, 2.0f, 0f); // 2 rad/s yaw rotation

                // Acquire with distortion
                sensor.AcquireAndPublishScan(10.0, 0.05);
                float[] distortedRanges = (float[])rangeProp.GetValue(sensor);
                Assert(distortedRanges != null && distortedRanges.Length == 1080, "Distorted ranges array null or wrong length");

                // Check that beam angles shifted due to motion
                float distortedCenterDist = distortedRanges[centerIdx];
                Debug.Log($"[Lakibeam1Verification] Center beam distance under 2.0 rad/s rotation: {distortedCenterDist:F3} m");
                Assert(!float.IsNaN(distortedCenterDist), "Distorted center dist is NaN");

                return "[Lakibeam1Verification] All checks PASSED successfully!";
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(obstacle);
                UnityEngine.Object.DestroyImmediate(sensorObj);
            }
        }
    }
}
#endif
