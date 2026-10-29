using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using Debug = UnityEngine.Debug;

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
        public const double ImuStandardGravity = 9.80665; // m/s² per g, independent of scene gravity.
        public const float MaxPointCloudPublishRateHz = 100f;
        public const float MaxImuPublishRateHz = 500f;
        [SerializeField] private string frameId = "livox_frame";

        [Header("Publishing Rates")]
        [Tooltip("PointCloud2 messages per simulation second (0.1–100 Hz). The 200,000 raw samples/s budget is divided between messages; time-resolved mesh poses update at the physics rate.")]
        [SerializeField, Range(0.1f, MaxPointCloudPublishRateHz)] private float pointCloudPublishRateHz = 10f;
        [Tooltip("IMU measurements per simulation second (0.1–500 Hz). Changes also apply during Play mode. Completed measurements are sent without an additional delivery buffer; timestamps are preserved.")]
        [SerializeField, Range(0.1f, MaxImuPublishRateHz)] private float imuPublishRateHz = 200f;

        [Header("Scene Capture")]
        [Tooltip("Measure each point at its simulation timestamp using interpolated poses and rendered mesh triangles. Disable only for legacy GPU scene snapshots.")]
        [SerializeField] private bool simulateMotionDistortion = true;
        [SerializeField] private bool useGpu = true;
        [SerializeField, Range(64, 2048)] private int depthResolution = 1024;
        [Tooltip("Pose reference only. All robot surfaces remain visible to LiDAR.")]
        [SerializeField] private Transform baseFrame;
        [Tooltip("ROS base_link coordinates, matching the external base_link -> livox_frame TF.")]
        [SerializeField] private Vector3 sensorPositionRos = new Vector3(0.24f, -0.25f, 1.09f);

        // Published raw points stay in the instantaneous LiDAR frame. Deskew belongs
        // to downstream consumers; the default acquisition includes scan motion.
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
        private Mid360TemporalScene _temporalScene;
        private Rigidbody _baseBody;
        private Vector3 _previousBasePosition;
        private Quaternion _previousBaseRotation;
        private double _previousCaptureTime;
        private bool _haveCapture;
        private double _sampleEpoch;
        private long _sampleNumber;
        private ScanFrame _acquiringScan;
        private int _acquiringCount;
        private double _nextScanTime;
        private float _activePointCloudRateHz;
        private double _fractionalScanPoints;
        private int _nextScanPointCount;
        private volatile float _requestedImuRateHz = 200f;
        private readonly object _imuLock = new object();
        private readonly Mid360ImuSampler _imuSampler = new Mid360ImuSampler();
        private Vector3 _imuPreviousPosition;
        private Quaternion _imuPreviousRotation;
        private double _imuPreviousTime;
        private bool _imuHavePose;
        private readonly Queue<sensor_msgs.msg.Imu> _completedImus = new Queue<sensor_msgs.msg.Imu>();
        private bool _destroying;

        public bool UsingGpu => _temporalScene != null ? _temporalScene.UsingGpu : _sceneSampler != null && _sceneSampler.UsingGpu;
        public long PublishedScanCount { get; private set; }
        public int LastPublishedPointCount { get; private set; }
        public long GeneratedImuCount { get; private set; }
        public long PublishedImuCount { get; private set; }
        public double LastGeneratedImuTimeSec { get; private set; }
        public double LastPublishedImuTimeSec { get; private set; }
        public double LastPublishedScanEndTimeSec { get; private set; }
        // Simulation-time age at publication, measured from the IMU sample or
        // the final LiDAR point (not the start of its accumulation window).
        public double LastImuDeliveryDelaySec { get; private set; }
        public double LastScanDeliveryDelaySec { get; private set; }
        public int PendingImuCount { get { lock (_imuLock) return _completedImus.Count; } }
        public int PendingScanCount { get { lock (_scanLock) return _completedScans.Count; } }
        public int PendingGpuIntervalCount => _temporalScene?.PendingCount ?? 0;
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
            ToRosTime(CurrentSimTimeSec, out sec, out nanosec);
        }

        public static void ToRosTime(double time, out int sec, out uint nanosec)
        {
            if (double.IsNaN(time) || double.IsInfinity(time)) throw new ArgumentOutOfRangeException(nameof(time));
            long ns = checked((long)Math.Round(time * 1e9, MidpointRounding.AwayFromZero));
            long seconds = Math.DivRem(ns, 1000000000L, out long fraction);
            if (fraction < 0) { seconds--; fraction += 1000000000L; }
            sec = checked((int)seconds);
            nanosec = (uint)fraction;
        }

        private static readonly object TimeLock = new object();
        private static double _simTimeSec = 10.0;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<rosgraph_msgs.msg.Clock> _clockPub;
        private IPublisher<sensor_msgs.msg.PointCloud2> _cloudPub;
        private IPublisher<sensor_msgs.msg.Imu> _imuPub;
        private Action<sensor_msgs.msg.Imu> _sendImu;
        private Action<sensor_msgs.msg.PointCloud2> _sendCloud;

        // Ray directions in ROS base_link (X forward, Y left, Z up) and Livox frame
        private Vector3[] _rayDirBaseRos;
        private Vector3[] _rayDirLivoxRos;
        private int _totalRays = 0;
        private int _scanStartIndex = 0;

        private double _lastStepSimSec = 10.0;
        private double _clockAccumSec;
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
            // At entry the preceding physics step is complete. Capture before
            // advancing the clock for the next simulation step, never in LateUpdate.
            double measurementTime = CurrentSimTimeSec;
            if (_rayDirBaseRos != null) CaptureImuInterval(measurementTime);
            if (_temporalScene != null) CaptureTemporalInterval(measurementTime);
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
            _baseBody = baseFrame != null ? baseFrame.GetComponent<Rigidbody>() : null;
            if (simulateMotionDistortion)
            {
                _temporalScene = new Mid360TemporalScene(useGpu);
                Debug.Log($"[LivoxMid360Sim] Time-resolved mesh BVH acquisition: {(_temporalScene.UsingGpu ? "GPU compute" : "CPU triangles")}; per-point sensor and mesh poses interpolated between physics steps.");
            }
            else
            {
                _sceneSampler = new Mid360SceneSampler(useGpu, depthResolution);
                Debug.Log($"[LivoxMid360Sim] Legacy snapshot capture: {(_sceneSampler.UsingGpu ? "GPU depth + compute" : "CPU physics")}; no motion distortion.");
            }
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
            _temporalScene?.Dispose();
            lock (_scanLock) _completedScans.Clear();
            lock (_imuLock) _completedImus.Clear();
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
                _sendImu = _imuPub.Publish;
                _sendCloud = _cloudPub.Publish;
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
            if (_node == null) InitROS2();
        }

        // Run after drive/opponent/cloth Update so capture sees their displayed poses.
        private void LateUpdate()
        {
            _temporalScene?.PollReadback();
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

        private void CaptureTemporalInterval(double now)
        {
            Transform reference = baseFrame != null ? baseFrame : transform;
            Vector3 position = _baseBody != null ? _baseBody.position : reference.position;
            Quaternion rotation = _baseBody != null ? _baseBody.rotation : reference.rotation;
            _temporalScene.Capture();
            if (!_haveCapture)
            {
                _sampleEpoch = now;
                _haveCapture = true;
            }
            else
            {
                long firstNumber = _sampleNumber;
                int firstPattern = _scanStartIndex;
                var rays = new List<Mid360TemporalScene.TimedRay>();
                while (true)
                {
                    double time = _sampleEpoch + (double)_sampleNumber / PointRate;
                    if (time >= now - 1e-10) break;
                    float fraction = (float)((time - _previousCaptureTime) / (now - _previousCaptureTime));
                    GetTimedRay(_previousBasePosition, _previousBaseRotation, position, rotation,
                        fraction, sensorPositionRos, _rayDirBaseRos[_scanStartIndex], out var origin, out var direction);
                    rays.Add(new Mid360TemporalScene.TimedRay
                    {
                        Origin = new Vector4(origin.x, origin.y, origin.z, fraction),
                        Direction = new Vector4(direction.x, direction.y, direction.z, 0)
                    });
                    _sampleNumber++;
                    _scanStartIndex = (_scanStartIndex + 1) % _totalRays;
                }
                double firstTime = _sampleEpoch + (double)firstNumber / PointRate;
                float rate = PointCloudPublishRateHz;
                // Submission owns this interval's immutable mesh/pose snapshots.
                // Readbacks are delivered in order, never resampled at a newer pose.
                _temporalScene.Submit(rays.ToArray(), MinRange, MaxRange,
                    returns => AppendTemporalSamples(returns, firstPattern, firstTime, rate));
            }
            _previousBasePosition = position;
            _previousBaseRotation = rotation;
            _previousCaptureTime = now;
        }

        private void AppendTemporalSamples(Vector2[] returns, int firstPattern, double firstTime, float rate)
        {
            if (_destroying) return;
            for (int i = 0; i < returns.Length; i++)
            {
                if (_acquiringScan == null)
                {
                    if (rate != _activePointCloudRateHz)
                    {
                        _activePointCloudRateHz = rate;
                        _fractionalScanPoints = 0;
                    }
                    int count = CalculateScanPointCount(rate, ref _fractionalScanPoints);
                    _acquiringScan = new ScanFrame
                    {
                        Returns = new Vector2[count], PointCount = count,
                        StartIndex = (firstPattern + i) % _totalRays,
                        Period = (double)count / PointRate,
                        EndTime = firstTime + (double)(i + count) / PointRate
                    };
                    _acquiringCount = 0;
                }
                _acquiringScan.Returns[_acquiringCount++] = returns[i];
                if (_acquiringCount == _acquiringScan.PointCount)
                {
                    lock (_scanLock) _completedScans.Enqueue(_acquiringScan);
                    _acquiringScan = null;
                }
            }
        }

        public static void GetTimedRay(Vector3 previousPosition, Quaternion previousRotation,
            Vector3 position, Quaternion rotation, float fraction, Vector3 sensorOffsetRos,
            Vector3 directionBaseRos, out Vector3 origin, out Vector3 direction)
        {
            Quaternion pose = Quaternion.Slerp(previousRotation, rotation, fraction);
            origin = Vector3.Lerp(previousPosition, position, fraction) +
                pose * new Vector3(-sensorOffsetRos.y, sensorOffsetRos.z, sensorOffsetRos.x);
            direction = pose * new Vector3(-directionBaseRos.y, directionBaseRos.z, directionBaseRos.x);
        }

        private void OnBackgroundTick()
        {
            if (_clockPub == null || _imuPub == null || _cloudPub == null) return;

            double currentSim = CurrentSimTimeSec;
            double delta = currentSim - _lastStepSimSec;
            if (delta > 0)
            {
                _lastStepSimSec = currentSim;

                _clockAccumSec += delta;
                if (_clockAccumSec >= 0.01)
                {
                    _clockAccumSec %= 0.01;
                    PublishClock(currentSim);
                }
            }

            PublishReadyMeasurements(currentSim, _sendImu, _sendCloud);
        }

        private void PublishReadyMeasurements(double horizon, Action<sensor_msgs.msg.Imu> sendImu,
            Action<sensor_msgs.msg.PointCloud2> sendCloud)
        {
            // Match livox_ros_driver2: drain completed acquisition queues in order.
            // Their device-rate clocks already live in the physics samplers. A
            // second wall-time clock here permanently retains any readback/Editor
            // stall as latency, even after the stall has ended.
            PublishReadyImus(horizon, sendImu);
            int ready;
            lock (_scanLock) ready = _completedScans.Count;
            for (int i = 0; i < ready; i++)
            {
                ScanFrame scan;
                lock (_scanLock)
                {
                    if (_completedScans.Peek().EndTime > horizon + 1e-9) break;
                    scan = _completedScans.Dequeue();
                }
                // A GPU result can arrive while this executor is already draining.
                // Send available IMU coverage before each cloud as well as at entry.
                // DDS/subscriber callback ordering across topics is not guaranteed.
                PublishReadyImus(horizon, sendImu);
                PublishPointCloud(scan, sendCloud);
            }
        }

        private void PublishReadyImus(double horizon, Action<sensor_msgs.msg.Imu> send)
        {
            // Snapshot the count so a continuously advancing producer cannot hold
            // the shared ROS executor indefinitely. Never hold a queue lock in DDS.
            int ready;
            lock (_imuLock) ready = _completedImus.Count;
            for (int i = 0; i < ready; i++)
            {
                sensor_msgs.msg.Imu sample;
                double time;
                lock (_imuLock)
                {
                    sample = _completedImus.Peek();
                    time = sample.Header.Stamp.Sec + sample.Header.Stamp.Nanosec * 1e-9;
                    if (time > horizon + 1e-9) break;
                    _completedImus.Dequeue();
                }
                send(sample);
                LastPublishedImuTimeSec = time;
                LastImuDeliveryDelaySec = Math.Max(0, CurrentSimTimeSec - time);
                PublishedImuCount++;
            }
        }

        private void PublishClock(double simTimeSec)
        {
            ToRosTime(simTimeSec, out int sec, out uint nanosec);
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

        private void CaptureImuInterval(double time)
        {
            Transform reference = baseFrame != null ? baseFrame : transform;
            Quaternion rotation = _baseBody != null ? _baseBody.rotation : reference.rotation;
            Vector3 position = _baseBody != null ? _baseBody.position : reference.position;
            Vector3 velocity = Vector3.zero, omega = Vector3.zero;
            if (_baseBody != null)
            {
                // Rigidbody velocities refer to its centre of mass, not base_link.
                velocity = _baseBody.linearVelocity;
                omega = _baseBody.angularVelocity;
            }
            else if (_imuHavePose && time > _imuPreviousTime)
            {
                float dt = (float)(time - _imuPreviousTime);
                velocity = (position - _imuPreviousPosition) / dt;
                Quaternion change = rotation * Quaternion.Inverse(_imuPreviousRotation);
                change.ToAngleAxis(out float angle, out Vector3 axis);
                if (angle > 180) angle -= 360;
                if (Mathf.Abs(angle) > 1e-6f) omega = axis * (angle * Mathf.Deg2Rad / dt);
            }
            Vector3 chipBaseRos = sensorPositionRos + RotateLivoxToBase(0.011f, 0.02329f, -0.04412f);
            Vector3 arm = new Vector3(-chipBaseRos.y, chipBaseRos.z, chipBaseRos.x);
            if (_baseBody != null) arm -= _baseBody.centerOfMass;
            _imuSampler.Capture(new Mid360ImuSampler.State
            {
                Time = time, Rotation = rotation, VelocityWorld = velocity, AngularVelocityWorld = omega
            }, NormalizePublishRate(imuPublishRateHz, 200f, MaxImuPublishRateHz), arm, Physics.gravity, sample =>
            {
                var message = BuildImuMessage(sample, frameId);
                lock (_imuLock) _completedImus.Enqueue(message);
                GeneratedImuCount++;
                LastGeneratedImuTimeSec = sample.Time;
            });
            _imuPreviousPosition = position; _imuPreviousRotation = rotation;
            _imuPreviousTime = time; _imuHavePose = true;
        }

        public static sensor_msgs.msg.Imu BuildImuMessage(Mid360ImuSampler.Sample sample, string frame)
        {
            ToRosTime(sample.Time, out int sec, out uint nanosec);
            var imu = new sensor_msgs.msg.Imu();
            imu.Header.Frame_id = frame;
            imu.Header.Stamp.Sec = sec; imu.Header.Stamp.Nanosec = nanosec;
            Quaternion inverse = Quaternion.Inverse(sample.Rotation);
            Vector3 angular = inverse * sample.AngularVelocityWorld;
            // Angular velocity is an axial vector: the handedness conversion
            // has the opposite sign from the polar acceleration conversion.
            Vector3 angularLivox = RotateBaseToLivox(-angular.z, angular.x, -angular.y);
            Vector3 acceleration = inverse * sample.SpecificForceWorld;
            Vector3 accelerationLivox = RotateBaseToLivox(acceleration.z, -acceleration.x, acceleration.y);
            imu.Angular_velocity.X = angularLivox.x; imu.Angular_velocity.Y = angularLivox.y; imu.Angular_velocity.Z = angularLivox.z;
            // livox_ros_driver2 forwards the device's accelerometer values in g.
            // Keep physics/sampling in SI and convert only at the ROS adapter.
            imu.Linear_acceleration.X = accelerationLivox.x / ImuStandardGravity;
            imu.Linear_acceleration.Y = accelerationLivox.y / ImuStandardGravity;
            imu.Linear_acceleration.Z = accelerationLivox.z / ImuStandardGravity;
            imu.Orientation_covariance[0] = -1;
            return imu;
        }

        private void PublishPointCloud(ScanFrame scan, Action<sensor_msgs.msg.PointCloud2> send)
        {
            var cloud = BuildRawPointCloud(scan.Returns, _rayDirLivoxRos, scan.StartIndex,
                scan.EndTime - scan.Period, frameId, scan.ReturnOffset, scan.PointCount);
            send(cloud);
            LastPublishedScanEndTimeSec = scan.EndTime - 1.0 / PointRate;
            LastScanDeliveryDelaySec = Math.Max(0, CurrentSimTimeSec - LastPublishedScanEndTimeSec);
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
            ToRosTime(startSimTimeSec, out int headerSec, out uint headerNanosec);
            // livox_ros_driver2 xfer_format=0: packed LivoxPointXyzrtlt.
            // https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/comm.h
            const int pointStep = 26;
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
                pointWriter.Write((byte)0); // Normal ideal return. No synthetic weather/noise classification.
                // The official MID-360/S adapter assigns packet point index % 4.
                // Its 96-point packets and our 800,000-ray pattern are divisible
                // by four, so the pattern index preserves phase across ROS windows.
                pointWriter.Write((byte)(rayIdx % 4));
                pointWriter.Write(stampNs); // Absolute simulation timestamp, nanoseconds.
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

            cloud.Fields = new sensor_msgs.msg.PointField[7];
            cloud.Fields[0] = new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 };
            cloud.Fields[1] = new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 };
            cloud.Fields[2] = new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 };
            cloud.Fields[3] = new sensor_msgs.msg.PointField { Name = "intensity", Offset = 12, Datatype = 7, Count = 1 };
            cloud.Fields[4] = new sensor_msgs.msg.PointField { Name = "tag", Offset = 16, Datatype = 2, Count = 1 };
            cloud.Fields[5] = new sensor_msgs.msg.PointField { Name = "line", Offset = 17, Datatype = 2, Count = 1 };
            cloud.Fields[6] = new sensor_msgs.msg.PointField { Name = "timestamp", Offset = 18, Datatype = 8, Count = 1 };

            return cloud;
        }
    }
}
