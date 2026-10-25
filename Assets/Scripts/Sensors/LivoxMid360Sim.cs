using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using RobotSim.Shooter;
using Debug = UnityEngine.Debug;

namespace RobotSim.Sensors
{
    public class LivoxMid360Sim : MonoBehaviour
    {
        // Fixed MID-360S acquisition specifications; old scene values for the former
        // 0.2 m / 40 m limits cannot silently override these device limits.
        public const float MinRange = 0.1f;
        public const float MaxRange = 100f;
        public const int PointRate = 200000;
        public const int PointsPerScan = 20000;
        public const double ScanPeriod = 0.1;
        public const double ImuPeriod = 0.005;
        [SerializeField] private string frameId = "livox_frame";

        [Header("Scene Capture")]
        [SerializeField] private bool useGpu = true;
        [SerializeField, Range(64, 2048)] private int depthResolution = 1024;
        [Tooltip("Pose reference only. All robot surfaces remain visible to LiDAR.")]
        [SerializeField] private Transform baseFrame;
        [Tooltip("ROS base_link coordinates, matching the external base_link -> livox_frame TF.")]
        [SerializeField] private Vector3 sensorPositionRos = new Vector3(0.24f, -0.25f, 1.09f);

        // Capture is a scene snapshot, without motion distortion. Per-ray timestamps
        // retain the existing scan-period convention for downstream localization.
        private sealed class ScanFrame
        {
            public Vector2[] Returns;
            public int StartIndex;
            public double EndTime;
            public double Period;
        }

        private Mid360SceneSampler _sceneSampler;
        private readonly object _scanLock = new object();
        private readonly Queue<ScanFrame> _completedScans = new Queue<ScanFrame>();
        private Vector3[] _worldDirections;
        private double _nextScanTime;
        private bool _destroying;

        public bool UsingGpu => _sceneSampler != null && _sceneSampler.UsingGpu;
        public long PublishedScanCount { get; private set; }
        public int LastPublishedPointCount { get; private set; }

        public static double CurrentSimTimeSec
        {
            get
            {
                lock (TimeLock)
                {
                    return _simTimeSec;
                }
            }
        }

        public static void GetSimTime(out int sec, out uint nanosec)
        {
            double t = CurrentSimTimeSec;
            sec = (int)t;
            nanosec = (uint)((t - sec) * 1e9);
        }

        private static readonly object TimeLock = new object();
        private static double _simTimeSec = 10.0;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<rosgraph_msgs.msg.Clock> _clockPub;
        private IPublisher<sensor_msgs.msg.PointCloud2> _cloudPub;
        private IPublisher<sensor_msgs.msg.Imu> _imuPub;

        // Ray directions in ROS base_link (X forward, Y left, Z up) and Livox frame
        private Vector3[] _rayDirBaseRos;
        private Vector3[] _rayDirLivoxRos;
        private int _totalRays = 0;
        private int _scanStartIndex = 0;

        private Stopwatch _wallClock;
        private double _lastStepWallSec = 0.0;
        private double _imuAccumSec = 0.0;
        private double _clockAccumSec;
        private double _lastImuTime;
        private float _lastImuAngularVelocity;
        private bool _executableRegistered = false;

        private void Awake()
        {
            Application.runInBackground = true;
            Time.fixedDeltaTime = 0.01f;
            Time.maximumDeltaTime = 0.10f;
            lock (TimeLock)
            {
                _simTimeSec = 10.0;
            }
            _wallClock = Stopwatch.StartNew();
            _lastStepWallSec = 0.0;
        }

        private void Start()
        {
            LoadPattern();
            if (baseFrame == null)
            {
                var drive = GetComponentInParent<KinematicOmniDrive>();
                if (drive != null) baseFrame = drive.transform;
            }
            _sceneSampler = new Mid360SceneSampler(useGpu, depthResolution);
            Debug.Log($"[LivoxMid360Sim] Scene capture: {(_sceneSampler.UsingGpu ? "GPU depth + compute" : "CPU physics")}");
            InitROS2();
        }

        private void OnDestroy()
        {
            _destroying = true;
            if (_ros2Unity != null && _executableRegistered)
            {
                _ros2Unity.UnregisterExecutable(OnBackgroundTick);
                _executableRegistered = false;
            }
            _sceneSampler?.Dispose();
            lock (_scanLock) _completedScans.Clear();
        }

        private void LoadPattern()
        {
            string path = Path.Combine(Application.streamingAssetsPath, "mid360_pattern.bin");
            if (!File.Exists(path))
            {
                throw new FileNotFoundException("MID360 scan pattern is required; a synthetic substitute is not a calibrated MID-360S scan.", path);
            }

            byte[] bytes = File.ReadAllBytes(path);
            if (bytes.Length == 0 || bytes.Length % (sizeof(float) * 2) != 0)
                throw new InvalidDataException("MID360 scan pattern must contain azimuth/elevation float32 pairs.");
            _totalRays = bytes.Length / (sizeof(float) * 2);
            _rayDirBaseRos = new Vector3[_totalRays];
            _rayDirLivoxRos = new Vector3[_totalRays];

            using (BinaryReader reader = new BinaryReader(new MemoryStream(bytes)))
            {
                for (int i = 0; i < _totalRays; i++)
                {
                    float azimuth = reader.ReadSingle();
                    float elevation = reader.ReadSingle();
                    if (float.IsNaN(azimuth) || float.IsInfinity(azimuth) ||
                        float.IsNaN(elevation) || float.IsInfinity(elevation))
                        throw new InvalidDataException("MID360 scan pattern contains a non-finite angle.");

                    float cosEl = Mathf.Cos(elevation);
                    float rx = cosEl * Mathf.Cos(azimuth);
                    float ry = cosEl * Mathf.Sin(azimuth);
                    float rz = Mathf.Sin(elevation);

                    Vector3 dLivox = new Vector3(rx, ry, rz).normalized;
                    _rayDirLivoxRos[i] = dLivox;
                    _rayDirBaseRos[i] = RotateLivoxToBase(dLivox.x, dLivox.y, dLivox.z);
                }
            }
            Debug.Log($"[LivoxMid360Sim] Loaded {_totalRays} ray directions from mid360_pattern.bin");
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                _node = _ros2Unity.CreateNode("livox_mid360_sim");
                _clockPub = _node.CreatePublisher<rosgraph_msgs.msg.Clock>("/clock");
                _cloudPub = _node.CreatePublisher<sensor_msgs.msg.PointCloud2>("/livox/lidar");
                _imuPub = _node.CreatePublisher<sensor_msgs.msg.Imu>("/livox/imu");
                if (!_executableRegistered)
                {
                    _ros2Unity.RegisterExecutable(OnBackgroundTick);
                    _executableRegistered = true;
                }
                Debug.Log("[LivoxMid360Sim] ROS2 Publishers initialized for /clock, /livox/lidar, and /livox/imu");
            }
        }

        private void Update()
        {
            if (_node == null) InitROS2();
        }

        // Run after drive/opponent/cloth Update so capture sees their displayed poses.
        private void LateUpdate()
        {
            if (_sceneSampler == null || _sceneSampler.IsBusy || _cloudPub == null || _totalRays == 0) return;
            double now = CurrentSimTimeSec;
            if (now < _nextScanTime) return;
            double period = ScanPeriod;
            // Advance the deadline, rather than adding a period to this frame's
            // late arrival; otherwise a 25 FPS editor turns 10 Hz into about 8 Hz.
            if (_nextScanTime <= 0) _nextScanTime = now;
            _nextScanTime += period;
            if (_nextScanTime <= now) _nextScanTime = now + period;
            int count = PointsPerScan;
            if (_worldDirections == null || _worldDirections.Length != count) _worldDirections = new Vector3[count];
            int startIndex = _scanStartIndex;
            _scanStartIndex = (_scanStartIndex + count) % _totalRays;

            // Use the owning robot's actual Unity pose, matching the meshes captured
            // this frame, rather than racing the background kinematics state.
            Transform baseTransform = baseFrame != null ? baseFrame : transform;
            Quaternion rotation = baseTransform.rotation;
            Vector3 origin = baseTransform.position + rotation * new Vector3(-sensorPositionRos.y, sensorPositionRos.z, sensorPositionRos.x);
            for (int i = 0; i < count; i++)
            {
                Vector3 d = _rayDirBaseRos[(startIndex + i) % _totalRays];
                _worldDirections[i] = rotation * new Vector3(-d.y, d.z, d.x);
            }
            _sceneSampler.TryScan(origin, _worldDirections, MinRange, MaxRange, ~0, returns =>
            {
                if (_destroying) return;
                // Preserve every completed raw scan. Do not replace or downsample it.
                lock (_scanLock)
                    _completedScans.Enqueue(new ScanFrame { Returns = returns, StartIndex = startIndex, EndTime = now, Period = period });
            });
        }

        private void OnBackgroundTick()
        {
            if (_clockPub == null || _imuPub == null || _cloudPub == null) return;

            double nowWall = _wallClock.Elapsed.TotalSeconds;
            double delta = nowWall - _lastStepWallSec;
            if (delta < 0.004) return;
            _lastStepWallSec = nowWall;

            if (delta > 0.05) delta = 0.01;

            double currentSim;
            lock (TimeLock)
            {
                _simTimeSec += delta;
                currentSim = _simTimeSec;
            }

            KinematicOmniDrive.StepKinematics((float)delta);
            SimpleClothShooter.StepClothPhysics((float)delta);

            _clockAccumSec += delta;
            if (_clockAccumSec >= 0.01)
            {
                _clockAccumSec %= 0.01;
                PublishClock(currentSim);
            }

            _imuAccumSec += delta;
            while (_imuAccumSec >= ImuPeriod)
            {
                _imuAccumSec -= ImuPeriod;
                PublishImu(currentSim - _imuAccumSec);
            }

            ScanFrame scan;
            lock (_scanLock)
            {
                scan = _completedScans.Count > 0 ? _completedScans.Dequeue() : null;
            }
            if (scan != null) PublishPointCloud(scan);
        }

        private void PublishClock(double simTimeSec)
        {
            int sec = (int)simTimeSec;
            uint nanosec = (uint)((simTimeSec - sec) * 1e9);
            rosgraph_msgs.msg.Clock clockMsg = new rosgraph_msgs.msg.Clock();
            clockMsg.Clock_.Sec = sec;
            clockMsg.Clock_.Nanosec = nanosec;
            _clockPub.Publish(clockMsg);
        }

        private static Vector3 RotateBaseToLivox(float bx, float by, float bz)
        {
            return new Vector3(
                 0.9629664f * bx + 0.0000000f * by + 0.2696215f * bz,
                -0.0030986f * bx - 0.9999342f * by + 0.0110668f * bz,
                 0.2696037f * bx - 0.0114924f * by - 0.9629030f * bz
            );
        }

        private static Vector3 RotateLivoxToBase(float lx, float ly, float lz)
        {
            return new Vector3(
                 0.9629664f * lx - 0.0030986f * ly + 0.2696037f * lz,
                 0.0000000f * lx - 0.9999342f * ly - 0.0114924f * lz,
                 0.2696215f * lx + 0.0110668f * ly - 0.9629030f * lz
            );
        }

        private void PublishImu(double simTimeSec)
        {
            int sec = (int)simTimeSec;
            uint nanosec = (uint)((simTimeSec - sec) * 1e9);

            sensor_msgs.msg.Imu imu = new sensor_msgs.msg.Imu();
            imu.Header.Frame_id = frameId;
            imu.Header.Stamp.Sec = sec;
            imu.Header.Stamp.Nanosec = nanosec;

            float wzBase = KinematicOmniDrive.SharedAngularVelocityRadZ;
            Vector3 omegaLivox = RotateBaseToLivox(0f, 0f, wzBase);
            imu.Angular_velocity.X = omegaLivox.x;
            imu.Angular_velocity.Y = omegaLivox.y;
            imu.Angular_velocity.Z = omegaLivox.z;

            Vector3 accelBaseUnity = KinematicOmniDrive.SharedLinearAccelBase;
            Vector3 localVelocityUnity = Quaternion.Inverse(KinematicOmniDrive.SharedRobotRotation) *
                KinematicOmniDrive.SharedLinearVelocityWorld;
            Vector3 localVelocityRos = new Vector3(localVelocityUnity.z, -localVelocityUnity.x, localVelocityUnity.y);
            Vector3 omegaBase = new Vector3(0, 0, wzBase);
            float alpha = _lastImuTime > 0 && simTimeSec > _lastImuTime
                ? (wzBase - _lastImuAngularVelocity) / (float)(simTimeSec - _lastImuTime) : 0f;
            _lastImuTime = simTimeSec;
            _lastImuAngularVelocity = wzBase;
            // MID-360S manual: IMU chip is (11.0, 23.29, -44.12) mm in LiDAR frame.
            Vector3 imuLeverArm = sensorPositionRos + RotateLivoxToBase(0.011f, 0.02329f, -0.04412f);
            Vector3 specificForce = new Vector3(accelBaseUnity.z, -accelBaseUnity.x, accelBaseUnity.y + 9.81f)
                + Vector3.Cross(omegaBase, localVelocityRos)
                + Vector3.Cross(new Vector3(0, 0, alpha), imuLeverArm)
                + Vector3.Cross(omegaBase, Vector3.Cross(omegaBase, imuLeverArm));
            Vector3 accLivox = RotateBaseToLivox(specificForce.x, specificForce.y, specificForce.z);
            imu.Linear_acceleration.X = accLivox.x;
            imu.Linear_acceleration.Y = accLivox.y;
            imu.Linear_acceleration.Z = accLivox.z;
            // The hardware outputs gyroscope/accelerometer measurements, not an attitude estimate.
            imu.Orientation_covariance[0] = -1;

            _imuPub.Publish(imu);
        }

        private void PublishPointCloud(ScanFrame scan)
        {
            var cloud = BuildRawPointCloud(scan.Returns, _rayDirLivoxRos, scan.StartIndex,
                scan.EndTime - scan.Period, frameId);
            _cloudPub.Publish(cloud);
            LastPublishedPointCount = (int)cloud.Width;
            PublishedScanCount++;
        }

        // ROS adapter only: preserve sample count/order, self hits, floor hits and
        // missing returns. No segmentation, point selection or feature extraction.
        public static sensor_msgs.msg.PointCloud2 BuildRawPointCloud(Vector2[] returns, Vector3[] pattern,
            int startIndex, double startSimTimeSec, string frame)
        {
            int count = returns.Length;
            int headerSec = (int)startSimTimeSec;
            uint headerNanosec = (uint)((startSimTimeSec - headerSec) * 1e9);
            const int pointStep = 25;
            byte[] pointData = new byte[count * pointStep];
            using var pointWriter = new BinaryWriter(new MemoryStream(pointData));
            for (int i = 0; i < count; i++)
            {
                float dist = returns[i].x;
                int rayIdx = (startIndex + i) % pattern.Length;
                // The device outputs (0,0,0) on no return. Keep its sample and time;
                // zero returns, floor hits and own-robot hits are never filtered out.
                Vector3 ptLivox = dist > 0f ? pattern[rayIdx] * dist : Vector3.zero;
                float intensity = dist > 0f ? returns[i].y : 0f;
                double stampNs = (startSimTimeSec + (double)i / PointRate) * 1e9;
                pointWriter.Write(ptLivox.x);
                pointWriter.Write(ptLivox.y);
                pointWriter.Write(ptLivox.z);
                pointWriter.Write(intensity);
                pointWriter.Write(stampNs);
                pointWriter.Write((byte)0); // Normal ideal return. No synthetic weather/noise classification.
            }

            sensor_msgs.msg.PointCloud2 cloud = new sensor_msgs.msg.PointCloud2();
            cloud.Header.Frame_id = frame;
            cloud.Header.Stamp.Sec = headerSec;
            cloud.Header.Stamp.Nanosec = headerNanosec;
            cloud.Height = 1;
            cloud.Width = (uint)count;
            cloud.Is_bigendian = false;
            cloud.Point_step = (uint)pointStep;
            cloud.Row_step = (uint)(count * pointStep);
            cloud.Is_dense = true;
            cloud.Data = pointData;

            cloud.Fields = new sensor_msgs.msg.PointField[6];
            cloud.Fields[0] = new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 };
            cloud.Fields[1] = new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 };
            cloud.Fields[2] = new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 };
            cloud.Fields[3] = new sensor_msgs.msg.PointField { Name = "intensity", Offset = 12, Datatype = 7, Count = 1 };
            cloud.Fields[4] = new sensor_msgs.msg.PointField { Name = "timestamp", Offset = 16, Datatype = 8, Count = 1 };

            cloud.Fields[5] = new sensor_msgs.msg.PointField { Name = "tag", Offset = 24, Datatype = 2, Count = 1 };

            return cloud;
        }
    }
}
