using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using RobotSim.Shooter;
using Debug = UnityEngine.Debug;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace RobotSim.Sensors
{
    public class LivoxMid360Sim : MonoBehaviour
    {
        // Device range/point rate and default publication settings. The frame and
        // IMU publication rates can be overridden independently in the Inspector.
        public const float MinRange = 0.1f;
        public const float MaxRange = 100f;
        public const int PointRate = 200000;
        public const int PointsPerScan = 20000;
        public const double ScanPeriod = 0.1;
        public const double ImuPeriod = 0.005;
        public const float MaxPointCloudPublishRateHz = 100f;
        public const float MaxImuPublishRateHz = 500f;
        [SerializeField] private string frameId = "livox_frame";

        [Header("Publishing Rates")]
        [Tooltip("PointCloud2 messages per simulation second (0.1–100 Hz). The 200,000 raw samples/s budget is divided between messages; scene poses update at the capture/render rate.")]
        [SerializeField, Range(0.1f, MaxPointCloudPublishRateHz)] private float pointCloudPublishRateHz = 10f;
        [Tooltip("IMU messages per simulation second (0.1–500 Hz). Changes also apply during Play mode. Uses 100 ms of simulation-time delivery buffering to smooth physics-frame batches; measurement timestamps are preserved.")]
        [SerializeField, Range(0.1f, MaxImuPublishRateHz)] private float imuPublishRateHz = 200f;

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
            public int ReturnOffset;
            public int PointCount;
            public int StartIndex;
            public double EndTime;
            public double Period;
        }

        private Mid360SceneSampler _sceneSampler;
        private readonly object _scanLock = new object();
        private readonly Queue<ScanFrame> _completedScans = new Queue<ScanFrame>();
        private Vector3[] _worldDirections;
        private double _nextScanTime;
        private float _activePointCloudRateHz;
        private double _fractionalScanPoints;
        private int _nextScanPointCount;
        private volatile float _requestedImuRateHz = 200f;
        private volatile float _requestedTimeScale = 1f;
        private long _lastCloudPaceTicks;
        private double _cloudPacingTimeSec;
        private double _nextCloudPublishTimeSec;
        private readonly Queue<sensor_msgs.msg.Imu> _completedImus = new Queue<sensor_msgs.msg.Imu>();
        private double _imuPacingTimeSec;
        private bool _imuPaceInitialized;
        // Physics delivers IMU samples in frame-sized batches. Buffer 100 ms
        // before sending so a short render-frame variation does not empty the
        // output queue and overflow a subscriber with the following catch-up burst.
        private const double ImuDeliveryBufferSec = 0.1;
        private float _activeImuRateHz = 200f;
        private bool _destroying;

        public bool UsingGpu => _sceneSampler != null && _sceneSampler.UsingGpu;
        public long PublishedScanCount { get; private set; }
        public int LastPublishedPointCount { get; private set; }
        public float PointCloudPublishRateHz => NormalizePublishRate(pointCloudPublishRateHz, 10f, MaxPointCloudPublishRateHz);
        public float ImuPublishRateHz => _requestedImuRateHz;

        private static float NormalizePublishRate(float hz, float fallback, float maximum)
        {
            if (float.IsNaN(hz) || float.IsInfinity(hz)) return fallback;
            return Mathf.Clamp(hz, 0.1f, maximum);
        }

        private void OnValidate()
        {
            pointCloudPublishRateHz = NormalizePublishRate(pointCloudPublishRateHz, 10f, MaxPointCloudPublishRateHz);
            imuPublishRateHz = NormalizePublishRate(imuPublishRateHz, 200f, MaxImuPublishRateHz);
            // The ROS executor reads only this managed value; never a Unity API.
            _requestedImuRateHz = imuPublishRateHz;
        }

        public static int CalculateScanPointCount(float publishRateHz, ref double fractionalPoints)
        {
            double budget = PointRate / (double)NormalizePublishRate(publishRateHz, 10f, MaxPointCloudPublishRateHz) + fractionalPoints;
            int count = (int)Math.Floor(budget + 1e-9);
            fractionalPoints = Math.Max(0, budget - count);
            return count;
        }

        private void ScheduleNextScan(double startTime, float hz)
        {
            _nextScanPointCount = CalculateScanPointCount(hz, ref _fractionalScanPoints);
            _nextScanTime = startTime + (double)_nextScanPointCount / PointRate;
        }

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

        private double _lastStepSimSec = 10.0;
        private double _imuAccumSec = 0.0;
        private double _clockAccumSec;
        private double _lastImuTime;
        private float _lastImuAngularVelocity;
        private bool _executableRegistered = false;

        private void Awake()
        {
            OnValidate();
            Application.runInBackground = true;
            Time.fixedDeltaTime = 0.01f;
            // Honor the project's catch-up budget. A sensor-imposed 100 ms cap
            // discarded physics time on slower render frames and lowered wall-time Hz.
            lock (TimeLock)
            {
                _simTimeSec = 10.0;
            }
            _lastStepSimSec = 10.0;
        }

        private void FixedUpdate()
        {
            // ROS time follows Unity physics, including pause and timeScale.
            lock (TimeLock) _simTimeSec += Time.fixedDeltaTime;
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
            _completedImus.Clear();
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
            _requestedImuRateHz = NormalizePublishRate(imuPublishRateHz, 200f, MaxImuPublishRateHz);
            _requestedTimeScale = Time.timeScale;
            if (_node == null) InitROS2();
        }

        // Run after drive/opponent/cloth Update so capture sees their displayed poses.
        private void LateUpdate()
        {
            if (_sceneSampler == null) return;
            _sceneSampler.PollReadback();
            if (_sceneSampler.IsBusy || _cloudPub == null || _totalRays == 0) return;
            double now = CurrentSimTimeSec;
            float hz = PointCloudPublishRateHz;
            if (_nextScanTime <= 0 || hz != _activePointCloudRateHz)
            {
                _activePointCloudRateHz = hz;
                _fractionalScanPoints = 0;
                ScheduleNextScan(now, hz);
            }
            if (now + 1e-9 >= _nextScanTime) CaptureScheduledScans(now, hz);
        }

        private void CaptureScheduledScans(double now, float hz)
        {
            double endTime = _nextScanTime;
            int nextCount = _nextScanPointCount;
            double fractionalPoints = _fractionalScanPoints;
            int startIndex = _scanStartIndex;
            int count = 0;
            var scans = new List<ScanFrame>();
            // One render/readback contains every due window. A readback slot is
            // an acquisition batch, not one ROS message: its latency must not
            // impose a messages-per-render-frame ceiling at high publish rates.
            while (now + 1e-9 >= endTime)
            {
                scans.Add(new ScanFrame
                {
                    ReturnOffset = count, PointCount = nextCount, StartIndex = startIndex,
                    EndTime = endTime, Period = (double)nextCount / PointRate
                });
                count = checked(count + nextCount);
                startIndex = (startIndex + nextCount) % _totalRays;
                nextCount = CalculateScanPointCount(hz, ref fractionalPoints);
                endTime += (double)nextCount / PointRate;
            }
            if (_worldDirections == null || _worldDirections.Length != count) _worldDirections = new Vector3[count];

            // Use the owning robot's actual Unity pose, matching the meshes captured
            // this frame, rather than racing the background kinematics state.
            Transform baseTransform = baseFrame != null ? baseFrame : transform;
            Quaternion rotation = baseTransform.rotation;
            Vector3 origin = baseTransform.position + rotation * new Vector3(-sensorPositionRos.y, sensorPositionRos.z, sensorPositionRos.x);
            for (int i = 0; i < count; i++)
            {
                Vector3 d = _rayDirBaseRos[(_scanStartIndex + i) % _totalRays];
                _worldDirections[i] = rotation * new Vector3(-d.y, d.z, d.x);
            }
            bool accepted = _sceneSampler.TryScan(origin, _worldDirections, MinRange, MaxRange, ~0, returns =>
            {
                if (_destroying) return;
                // Share the immutable readback array. Partition by exact sample
                // offsets, without copies, discarded returns or repeated clouds.
                lock (_scanLock)
                    foreach (var scan in scans)
                    {
                        scan.Returns = returns;
                        _completedScans.Enqueue(scan);
                    }
            });
            // A rejected submission must leave its windows available for retry.
            if (!accepted) return;
            _nextScanTime = endTime;
            _nextScanPointCount = nextCount;
            _fractionalScanPoints = fractionalPoints;
            _scanStartIndex = startIndex;
        }

        private void OnBackgroundTick()
        {
            if (_clockPub == null || _imuPub == null || _cloudPub == null) return;

            double currentSim = CurrentSimTimeSec;
            double delta = currentSim - _lastStepSimSec;
            if (delta > 0)
            {
                _lastStepSimSec = currentSim;

                // Cloth uses the same elapsed simulation time, with stable substeps.
                double remaining = delta;
                while (remaining > 1e-8)
                {
                    float step = (float)Math.Min(remaining, 0.01);
                    SimpleClothShooter.StepClothPhysics(step);
                    remaining -= step;
                }

                _clockAccumSec += delta;
                if (_clockAccumSec >= 0.01)
                {
                    _clockAccumSec %= 0.01;
                    PublishClock(currentSim);
                }

                float imuHz = _requestedImuRateHz;
                if (imuHz != _activeImuRateHz)
                {
                    _activeImuRateHz = imuHz;
                    // Do not reinterpret an old, slow-rate remainder as past samples
                    // when increasing the rate from the Inspector.
                    _imuAccumSec = 0;
                }
                double imuPeriod = 1.0 / imuHz;
                _imuAccumSec += delta;
                while (_imuAccumSec + 1e-9 >= imuPeriod)
                {
                    _imuAccumSec = Math.Max(0, _imuAccumSec - imuPeriod);
                    QueueImuSample(currentSim - _imuAccumSec);
                }
            }

            // Readbacks deliver a batch of consecutive windows. Pace those raw
            // messages on the executor's monotonic clock so render-frame bursts
            // do not overrun subscribers' queues. Their ROS/sample stamps remain
            // simulation time. The pacing clock follows timeScale and cannot run
            // beyond physics time, including when the Editor is paused.
            long ticks = Stopwatch.GetTimestamp();
            double elapsed = _lastCloudPaceTicks > 0 ? (double)(ticks - _lastCloudPaceTicks) / Stopwatch.Frequency : 0;
            _lastCloudPaceTicks = ticks;
            float timeScale = _requestedTimeScale;
            _cloudPacingTimeSec = Math.Min(currentSim, _cloudPacingTimeSec + elapsed * timeScale);
            if (timeScale <= 0) return;
            PublishReadyImus(currentSim, elapsed, timeScale);
            int ready;
            lock (_scanLock)
            {
                ready = _completedScans.Count;
                // A long physics/Editor pause exhausts the buffer and lets the
                // pacing clock catch physics. Re-anchor on the next acquisition
                // so resuming does not permanently turn paced output into bursts.
                if (ready == 0 && _cloudPacingTimeSec + 1e-9 >= currentSim)
                    _nextCloudPublishTimeSec = 0;
                if (ready > 0 && _nextCloudPublishTimeSec <= 0)
                    _nextCloudPublishTimeSec = _cloudPacingTimeSec = _completedScans.Peek().EndTime;
            }
            for (int i = 0; i < ready; i++)
            {
                if (_cloudPacingTimeSec + 1e-9 < _nextCloudPublishTimeSec) break;
                ScanFrame scan;
                lock (_scanLock) scan = _completedScans.Dequeue();
                PublishPointCloud(scan);
                _nextCloudPublishTimeSec += scan.Period;
            }
        }

        private void PublishReadyImus(double currentSim, double elapsed, float timeScale)
        {
            // Only the executor touches this queue. Build each measurement at
            // its physics update; publication must not resample a newer pose.
            if (!_imuPaceInitialized && _completedImus.Count > 0)
            {
                var stamp = _completedImus.Peek().Header.Stamp;
                _imuPacingTimeSec = stamp.Sec + stamp.Nanosec * 1e-9 - ImuDeliveryBufferSec;
                _imuPaceInitialized = true;
            }
            else
                _imuPacingTimeSec = Math.Min(currentSim, _imuPacingTimeSec + elapsed * timeScale);

            while (_completedImus.Count > 0)
            {
                var sample = _completedImus.Peek();
                double sampleTime = sample.Header.Stamp.Sec + sample.Header.Stamp.Nanosec * 1e-9;
                if (sampleTime > _imuPacingTimeSec + 1e-9) break;
                _imuPub.Publish(_completedImus.Dequeue());
            }
            if (_completedImus.Count == 0 && _imuPacingTimeSec + 1e-9 >= currentSim)
                _imuPaceInitialized = false;
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

        private void QueueImuSample(double simTimeSec)
        {
            int sec = (int)simTimeSec;
            uint nanosec = (uint)((simTimeSec - sec) * 1e9);

            sensor_msgs.msg.Imu imu = new sensor_msgs.msg.Imu();
            imu.Header.Frame_id = frameId;
            imu.Header.Stamp.Sec = sec;
            imu.Header.Stamp.Nanosec = nanosec;

            var motion = KinematicOmniDrive.GetMotionState();
            float wzBase = motion.AngularVelocityRadZ;
            Vector3 omegaLivox = RotateBaseToLivox(0f, 0f, wzBase);
            imu.Angular_velocity.X = omegaLivox.x;
            imu.Angular_velocity.Y = omegaLivox.y;
            imu.Angular_velocity.Z = omegaLivox.z;

            Vector3 accelBaseUnity = motion.LinearAccelerationBase;
            Vector3 omegaBase = new Vector3(0, 0, wzBase);
            float alpha = _lastImuTime > 0 && simTimeSec > _lastImuTime
                ? (wzBase - _lastImuAngularVelocity) / (float)(simTimeSec - _lastImuTime) : 0f;
            _lastImuTime = simTimeSec;
            _lastImuAngularVelocity = wzBase;
            // MID-360S manual: IMU chip is (11.0, 23.29, -44.12) mm in LiDAR frame.
            Vector3 imuLeverArm = sensorPositionRos + RotateLivoxToBase(0.011f, 0.02329f, -0.04412f);
            Vector3 specificForce = new Vector3(accelBaseUnity.z, -accelBaseUnity.x, accelBaseUnity.y + 9.81f)
                + Vector3.Cross(new Vector3(0, 0, alpha), imuLeverArm)
                + Vector3.Cross(omegaBase, Vector3.Cross(omegaBase, imuLeverArm));
            Vector3 accLivox = RotateBaseToLivox(specificForce.x, specificForce.y, specificForce.z);
            imu.Linear_acceleration.X = accLivox.x;
            imu.Linear_acceleration.Y = accLivox.y;
            imu.Linear_acceleration.Z = accLivox.z;
            // The hardware outputs gyroscope/accelerometer measurements, not an attitude estimate.
            imu.Orientation_covariance[0] = -1;

            _completedImus.Enqueue(imu);
        }

        private void PublishPointCloud(ScanFrame scan)
        {
            var cloud = BuildRawPointCloud(scan.Returns, _rayDirLivoxRos, scan.StartIndex,
                scan.EndTime - scan.Period, frameId, scan.ReturnOffset, scan.PointCount);
            _cloudPub.Publish(cloud);
            LastPublishedPointCount = (int)cloud.Width;
            PublishedScanCount++;
        }

        // ROS adapter only: preserve sample count/order, self hits, floor hits and
        // missing returns. No segmentation, point selection or feature extraction.
        public static sensor_msgs.msg.PointCloud2 BuildRawPointCloud(Vector2[] returns, Vector3[] pattern,
            int startIndex, double startSimTimeSec, string frame, int returnOffset = 0, int count = -1)
        {
            if (count < 0) count = returns.Length - returnOffset;
            if (returnOffset < 0 || count < 0 || returnOffset > returns.Length - count)
                throw new ArgumentOutOfRangeException(nameof(returnOffset));
            int headerSec = (int)startSimTimeSec;
            uint headerNanosec = (uint)((startSimTimeSec - headerSec) * 1e9);
            const int pointStep = 25;
            byte[] pointData = new byte[count * pointStep];
            using var pointWriter = new BinaryWriter(new MemoryStream(pointData));
            for (int i = 0; i < count; i++)
            {
                float dist = returns[returnOffset + i].x;
                int rayIdx = (startIndex + i) % pattern.Length;
                // The device outputs (0,0,0) on no return. Keep its sample and time;
                // zero returns, floor hits and own-robot hits are never filtered out.
                Vector3 ptLivox = dist > 0f ? pattern[rayIdx] * dist : Vector3.zero;
                float intensity = dist > 0f ? returns[returnOffset + i].y : 0f;
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
