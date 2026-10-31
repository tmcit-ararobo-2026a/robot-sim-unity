using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using RobotSim.Core;
using Debug = UnityEngine.Debug;

namespace RobotSim.Sensors
{
    /// <summary>
    /// Simulates a 2D planar LiDAR (Lakibeam 1) on the robot.
    /// Performs beam-wise raycasting, supports realistic motion distortion (deskew testable),
    /// and publishes sensor_msgs/LaserScan to ROS 2.
    /// </summary>
    public class Lakibeam1Sim : MonoBehaviour
    {
        [Header("ROS 2 Topic & Frame")]
        [SerializeField] private string topic = "/lakibeam/scan";
        [SerializeField] private string frameId = "lakibeam_frame";

        [Header("LiDAR Specs")]
        [Tooltip("Scan publish rate in Hz. Default is 20 Hz (0.05s period).")]
        [SerializeField, Range(1f, 50f)] private float scanRateHz = 20.0f;
        [Tooltip("Minimum angle in degrees (-180 = backward/front in sensor frame).")]
        [SerializeField] private float minAngleDeg = -180.0f;
        [Tooltip("Maximum angle in degrees (+180 = full 360 deg FOV).")]
        [SerializeField] private float maxAngleDeg = 180.0f;
        [Tooltip("Number of range measurement beams per complete revolution.")]
        [SerializeField, Range(180, 2880)] private int numBeams = 1080;
        [SerializeField] private float minRange = 0.05f;
        [SerializeField] private float maxRange = 25.0f;

        [Header("Motion Distortion")]
        [Tooltip("Whether to simulate beam-by-beam acquisition motion distortion based on robot velocity.")]
        [SerializeField] private bool simulateMotionDistortion = true;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<sensor_msgs.msg.LaserScan> _scanPub;
        private Action<sensor_msgs.msg.LaserScan> _sendScan;

        private Rigidbody _baseBody;
        private Transform _baseFrame;
        private HashSet<Collider> _ignoredColliders;

        private double _nextScanStartTime = -1.0;
        private float[] _ranges;
        private float[] _intensities;

        public string Topic => topic;
        public string FrameId => frameId;
        public float ScanRateHz => scanRateHz;
        public int NumBeams => numBeams;

        private void Awake()
        {
            _ranges = new float[numBeams];
            _intensities = new float[numBeams];
        }

        private void Start()
        {
            var drive = GetComponentInParent<KinematicOmniDrive>();
            if (drive != null)
            {
                _baseFrame = drive.transform;
                _baseBody = drive.GetComponent<Rigidbody>();
            }
            else
            {
                _baseFrame = transform.root;
                _baseBody = _baseFrame.GetComponent<Rigidbody>();
            }

            // Collect colliders on the robot body to ignore them during raycasting
            _ignoredColliders = new HashSet<Collider>(transform.root.GetComponentsInChildren<Collider>(true));

            InitROS2();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                _node = _ros2Unity.CreateNode("lakibeam1_sim");
                _scanPub = _node.CreatePublisher<sensor_msgs.msg.LaserScan>(topic);
                _sendScan = _scanPub.Publish;
                Debug.Log($"[Lakibeam1Sim] ROS 2 LaserScan publisher initialized for topic '{topic}' in frame '{frameId}'");
            }
        }

        private void Update()
        {
            if (_node == null) InitROS2();
        }

        private void LateUpdate()
        {
            if (_scanPub == null) return;

            double now = LivoxMid360Sim.CurrentSimTimeSec;
            double scanPeriod = 1.0 / Mathf.Clamp(scanRateHz, 1.0f, 100.0f);

            if (_nextScanStartTime < 0)
            {
                _nextScanStartTime = now;
            }

            // If a scan interval is complete, acquire and publish
            if (now >= _nextScanStartTime + scanPeriod)
            {
                double scanStart = _nextScanStartTime;
                _nextScanStartTime += scanPeriod;
                // If fell behind, resync to current time
                if (now > _nextScanStartTime + scanPeriod)
                {
                    _nextScanStartTime = now;
                }

                AcquireAndPublishScan(scanStart, scanPeriod);
            }
        }

        public void AcquireAndPublishScan(double scanStartTime, double scanPeriod)
        {
            int beams = numBeams;
            if (_ranges == null || _ranges.Length != beams)
            {
                _ranges = new float[beams];
                _intensities = new float[beams];
            }

            float minRad = minAngleDeg * Mathf.Deg2Rad;
            float maxRad = maxAngleDeg * Mathf.Deg2Rad;
            float angleInc = (maxRad - minRad) / beams;
            float timeInc = (float)(scanPeriod / beams);

            Vector3 sensorPos = transform.position;
            Quaternion sensorRot = transform.rotation;

            // Get robot velocity in world coordinates if motion distortion is simulated
            Vector3 worldLinVel = Vector3.zero;
            float angVelRadZ = 0f;
            if (simulateMotionDistortion)
            {
                if (_baseBody != null)
                {
                    worldLinVel = _baseBody.linearVelocity;
                    angVelRadZ = _baseBody.angularVelocity.y;
                }
                else
                {
                    var state = KinematicOmniDrive.GetMotionState();
                    worldLinVel = state.LinearVelocityWorld;
                    angVelRadZ = state.AngularVelocityRadZ;
                }
            }

            double currentSimTime = LivoxMid360Sim.CurrentSimTimeSec;

            for (int i = 0; i < beams; i++)
            {
                float beamAngle = minRad + i * angleInc;
                double beamTime = scanStartTime + i * timeInc;
                float dt = (float)(beamTime - currentSimTime);

                // Compute beam ray pose
                Vector3 origin = sensorPos;
                Quaternion rot = sensorRot;

                if (simulateMotionDistortion && Mathf.Abs(dt) > 1e-5f)
                {
                    origin += worldLinVel * dt;
                    rot = Quaternion.AngleAxis(angVelRadZ * dt * Mathf.Rad2Deg, Vector3.up) * sensorRot;
                }

                // In ROS sensor frame (REP-103):
                // x = cos(angle), y = sin(angle), z = 0
                // In Unity sensor coordinates:
                // Unity forward (+Z) is ROS +X, Unity left (-X) is ROS +Y
                Vector3 localDir = new Vector3(-Mathf.Sin(beamAngle), 0f, Mathf.Cos(beamAngle));
                Vector3 worldDir = rot * localDir;

                RaycastHit[] hits = Physics.RaycastAll(origin, worldDir, maxRange, ~0, QueryTriggerInteraction.Ignore);
                float bestDist = float.PositiveInfinity;

                for (int h = 0; h < hits.Length; h++)
                {
                    Collider col = hits[h].collider;
                    if (_ignoredColliders != null && _ignoredColliders.Contains(col))
                        continue;
                    if (hits[h].distance < bestDist)
                    {
                        bestDist = hits[h].distance;
                    }
                }

                if (bestDist >= minRange && bestDist <= maxRange)
                {
                    _ranges[i] = bestDist;
                    _intensities[i] = 100.0f;
                }
                else
                {
                    _ranges[i] = float.PositiveInfinity;
                    _intensities[i] = 0.0f;
                }
            }

            // Build LaserScan message
            var scan = new sensor_msgs.msg.LaserScan();
            LivoxMid360Sim.ToRosTime(scanStartTime, out int sec, out uint nanosec);
            scan.Header.Frame_id = frameId;
            scan.Header.Stamp.Sec = sec;
            scan.Header.Stamp.Nanosec = nanosec;
            scan.Angle_min = minRad;
            scan.Angle_max = maxRad;
            scan.Angle_increment = angleInc;
            scan.Time_increment = timeInc;
            scan.Scan_time = (float)scanPeriod;
            scan.Range_min = minRange;
            scan.Range_max = maxRange;
            scan.Ranges = _ranges;
            scan.Intensities = _intensities;

            _sendScan?.Invoke(scan);
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.cyan;
            Vector3 origin = transform.position;
            int step = Mathf.Max(1, numBeams / 36); // draw ~36 rays
            float minRad = minAngleDeg * Mathf.Deg2Rad;
            float angleInc = (maxAngleDeg - minAngleDeg) * Mathf.Deg2Rad / numBeams;

            for (int i = 0; i < numBeams; i += step)
            {
                float beamAngle = minRad + i * angleInc;
                Vector3 localDir = new Vector3(-Mathf.Sin(beamAngle), 0f, Mathf.Cos(beamAngle));
                Vector3 worldDir = transform.TransformDirection(localDir);
                Gizmos.DrawRay(origin, worldDir * Mathf.Min(maxRange, 2.0f));
            }
        }
    }
}

