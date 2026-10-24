using System;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using RobotSim.Shooter;
using Debug = UnityEngine.Debug;

namespace RobotSim.Sensors
{
    public class LivoxMid360Sim : MonoBehaviour
    {
        [Header("Lidar Specs")]
        [SerializeField] private float minRange = 0.2f;
        [SerializeField] private float maxRange = 40.0f;
        [SerializeField] private float scanRateHz = 10.0f;
        [SerializeField] private int pointsPerScan = 20000;
        [SerializeField] private string frameId = "livox_frame";

        private struct FieldBox
        {
            public float minX, maxX, minY, maxY, minZ, maxZ;
            public FieldBox(float cx, float cy, float zMin, float zMax, float halfX, float halfY)
            {
                minX = cx - halfX;
                maxX = cx + halfX;
                minY = cy - halfY;
                maxY = cy + halfY;
                minZ = zMin;
                maxZ = zMax;
            }
        }

        private struct FieldCylinder
        {
            public float cx, cy, r, rSq, minZ, maxZ;
            public FieldCylinder(float x, float y, float zMin, float zMax, float radius)
            {
                cx = x;
                cy = y;
                r = radius;
                rSq = radius * radius;
                minZ = zMin;
                maxZ = zMax;
            }
        }

        // Exact 3D geometry of NHK2026 field matching nhk2026_map.json and main.usd
        private static readonly FieldBox[] FieldBoxes = new FieldBox[]
        {
            new FieldBox( 0.000f,  5.775f, 0.000f, 0.024f, 5.400f, 0.075f),
            new FieldBox( 0.000f, -5.775f, 0.000f, 0.024f, 5.400f, 0.075f),
            new FieldBox(-5.325f,  0.000f, 0.000f, 0.024f, 0.075f, 5.700f),
            new FieldBox( 5.325f,  0.000f, 0.000f, 0.024f, 0.075f, 5.700f),
            new FieldBox( 0.000f,  5.712f, 0.024f, 0.150f, 5.400f, 0.012f),
            new FieldBox( 0.000f, -5.712f, 0.024f, 0.150f, 5.400f, 0.012f),
            new FieldBox(-5.262f,  0.000f, 0.024f, 0.150f, 0.012f, 5.700f),
            new FieldBox( 5.262f,  0.000f, 0.024f, 0.150f, 0.012f, 5.700f),
            new FieldBox( 0.000f,  0.000f, 0.000f, 0.200f, 5.250f, 0.300f),
            new FieldBox(-1.270f,  1.480f, 0.000f, 0.600f, 0.150f, 0.150f),
            new FieldBox(-1.270f, -1.480f, 0.000f, 0.600f, 0.150f, 0.150f),
            new FieldBox( 2.370f,  1.480f, 0.000f, 0.300f, 0.150f, 0.150f),
            new FieldBox( 2.370f, -1.480f, 0.000f, 0.300f, 0.150f, 0.150f),
            new FieldBox( 0.550f,  4.980f, 0.000f, 0.807f, 0.180f, 0.200f),
            new FieldBox( 0.550f, -4.980f, 0.000f, 0.807f, 0.180f, 0.200f),
            new FieldBox(-2.295f,  3.855f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox(-2.295f, -3.855f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox( 3.395f,  3.855f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox( 3.395f, -3.855f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox(-4.895f,  5.445f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox(-4.895f, -5.445f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox(-4.750f,  1.105f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox(-4.750f, -1.105f, 0.000f, 0.760f, 0.325f, 0.225f),
            new FieldBox( 0.550f,  3.025f, 0.000f, 0.180f, 0.195f, 0.195f),
            new FieldBox( 0.850f,  3.025f, 1.200f, 3.000f, 0.300f, 0.020f),
            new FieldBox( 0.550f, -3.025f, 0.000f, 0.180f, 0.195f, 0.195f),
            new FieldBox( 0.850f, -3.025f, 1.200f, 3.000f, 0.300f, 0.020f)
        };

        private static readonly FieldCylinder[] FieldCylinders = new FieldCylinder[]
        {
            new FieldCylinder( 0.550f,  0.870f, 0.000f, 0.255f, 0.1365f),
            new FieldCylinder( 0.550f, -0.870f, 0.000f, 0.255f, 0.1365f),
            new FieldCylinder(-1.270f,  1.480f, 0.600f, 0.855f, 0.1365f),
            new FieldCylinder(-1.270f, -1.480f, 0.600f, 0.855f, 0.1365f),
            new FieldCylinder( 2.370f,  1.480f, 0.300f, 0.555f, 0.1365f),
            new FieldCylinder( 2.370f, -1.480f, 0.300f, 0.555f, 0.1365f),
            new FieldCylinder( 0.550f,  3.025f, 0.180f, 3.000f, 0.0300f),
            new FieldCylinder( 0.550f, -3.025f, 0.180f, 3.000f, 0.0300f)
        };

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
        private double _scanAccumSec = 0.0;
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
            InitROS2();
        }

        private void OnDestroy()
        {
            if (_ros2Unity != null && _executableRegistered)
            {
                _ros2Unity.UnregisterExecutable(OnBackgroundTick);
                _executableRegistered = false;
            }
        }

        private void LoadPattern()
        {
            string path = Path.Combine(Application.streamingAssetsPath, "mid360_pattern.bin");
            if (!File.Exists(path))
            {
                Debug.LogWarning($"[LivoxMid360Sim] Pattern file not found at {path}, generating procedural pattern");
                GenerateProceduralPattern();
                return;
            }

            byte[] bytes = File.ReadAllBytes(path);
            _totalRays = bytes.Length / (sizeof(float) * 2);
            _rayDirBaseRos = new Vector3[_totalRays];
            _rayDirLivoxRos = new Vector3[_totalRays];

            using (BinaryReader reader = new BinaryReader(new MemoryStream(bytes)))
            {
                for (int i = 0; i < _totalRays; i++)
                {
                    float azimuth = reader.ReadSingle();
                    float elevation = reader.ReadSingle();

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

        private void GenerateProceduralPattern()
        {
            _totalRays = 100000;
            _rayDirBaseRos = new Vector3[_totalRays];
            _rayDirLivoxRos = new Vector3[_totalRays];
            for (int i = 0; i < _totalRays; i++)
            {
                float t = i * 0.0001f;
                float azimuth = (t * 7297.0f) % (2.0f * Mathf.PI);
                float elevation = Mathf.Lerp(-7.0f, 52.0f, (Mathf.Sin(t * 1310.0f) + 1.0f) * 0.5f) * Mathf.Deg2Rad;
                float cosEl = Mathf.Cos(elevation);
                float rx = cosEl * Mathf.Cos(azimuth);
                float ry = cosEl * Mathf.Sin(azimuth);
                float rz = Mathf.Sin(elevation);
                Vector3 dLivox = new Vector3(rx, ry, rz).normalized;
                _rayDirLivoxRos[i] = dLivox;
                _rayDirBaseRos[i] = RotateLivoxToBase(dLivox.x, dLivox.y, dLivox.z);
            }
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
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }
        }

        private void OnBackgroundTick()
        {
            if (_clockPub == null || _imuPub == null || _cloudPub == null) return;

            double nowWall = _wallClock.Elapsed.TotalSeconds;
            double delta = nowWall - _lastStepWallSec;
            if (delta < 0.008) return;
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

            PublishClock(currentSim);

            _imuAccumSec += delta;
            if (_imuAccumSec >= 0.0095)
            {
                _imuAccumSec = 0.0;
                PublishImu(currentSim);
            }

            double scanPeriod = 1.0 / scanRateHz;
            _scanAccumSec += delta;
            if (_scanAccumSec >= scanPeriod)
            {
                _scanAccumSec -= scanPeriod;
                if (_scanAccumSec > scanPeriod) _scanAccumSec = 0.0;
                PublishImu(currentSim);
                PublishPointCloud(currentSim, scanPeriod);
            }
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
            Vector3 accLivox = RotateBaseToLivox(accelBaseUnity.z, -accelBaseUnity.x, accelBaseUnity.y + 9.81f);
            imu.Linear_acceleration.X = accLivox.x;
            imu.Linear_acceleration.Y = accLivox.y;
            imu.Linear_acceleration.Z = accLivox.z;

            _imuPub.Publish(imu);
        }

        private static float RaycastFieldInMap(float ox, float oy, float oz, float dx, float dy, float dz, float minR, float maxR)
        {
            float bestT = maxR + 1.0f;

            // 1. Floor plane z = 0
            if (dz < -1e-5f)
            {
                float tFloor = -oz / dz;
                if (tFloor >= minR && tFloor < bestT)
                {
                    float fx = ox + tFloor * dx;
                    float fy = oy + tFloor * dy;
                    if (fx >= -5.40f && fx <= 5.40f && fy >= -5.85f && fy <= 5.85f)
                    {
                        bestT = tFloor;
                    }
                }
            }

            float invDx = Mathf.Abs(dx) > 1e-6f ? 1.0f / dx : (dx >= 0f ? 1e6f : -1e6f);
            float invDy = Mathf.Abs(dy) > 1e-6f ? 1.0f / dy : (dy >= 0f ? 1e6f : -1e6f);
            float invDz = Mathf.Abs(dz) > 1e-6f ? 1.0f / dz : (dz >= 0f ? 1e6f : -1e6f);

            // 2. Field Boxes
            int numBoxes = FieldBoxes.Length;
            for (int i = 0; i < numBoxes; i++)
            {
                FieldBox b = FieldBoxes[i];
                float tx1 = (b.minX - ox) * invDx;
                float tx2 = (b.maxX - ox) * invDx;
                float tmin = tx1 < tx2 ? tx1 : tx2;
                float tmax = tx1 > tx2 ? tx1 : tx2;

                float ty1 = (b.minY - oy) * invDy;
                float ty2 = (b.maxY - oy) * invDy;
                float tyMin = ty1 < ty2 ? ty1 : ty2;
                float tyMax = ty1 > ty2 ? ty1 : ty2;

                if (tyMin > tmin) tmin = tyMin;
                if (tyMax < tmax) tmax = tyMax;

                if (tmax < tmin || tmax < minR || tmin >= bestT) continue;

                float tz1 = (b.minZ - oz) * invDz;
                float tz2 = (b.maxZ - oz) * invDz;
                float tzMin = tz1 < tz2 ? tz1 : tz2;
                float tzMax = tz1 > tz2 ? tz1 : tz2;

                if (tzMin > tmin) tmin = tzMin;
                if (tzMax < tmax) tmax = tzMax;

                if (tmax >= tmin)
                {
                    float tHit = tmin >= minR ? tmin : tmax;
                    if (tHit >= minR && tHit < bestT)
                    {
                        bestT = tHit;
                    }
                }
            }

            // 3. Field Cylinders
            float a = dx * dx + dy * dy;
            if (a > 1e-6f)
            {
                float invA = 1.0f / a;
                int numCyls = FieldCylinders.Length;
                for (int i = 0; i < numCyls; i++)
                {
                    FieldCylinder c = FieldCylinders[i];
                    float ocx = ox - c.cx;
                    float ocy = oy - c.cy;
                    float halfB = ocx * dx + ocy * dy;
                    float cTerm = ocx * ocx + ocy * ocy - c.rSq;
                    float disc = halfB * halfB - a * cTerm;
                    if (disc < 0f) continue;

                    float sqrtDisc = Mathf.Sqrt(disc);
                    float t1 = (-halfB - sqrtDisc) * invA;
                    if (t1 >= minR && t1 < bestT)
                    {
                        float hz = oz + t1 * dz;
                        if (hz >= c.minZ && hz <= c.maxZ)
                        {
                            bestT = t1;
                        }
                    }
                }
            }

            return bestT <= maxR ? bestT : -1.0f;
        }

        private void PublishPointCloud(double endSimTimeSec, double scanPeriodSec)
        {
            if (_rayDirBaseRos == null || _totalRays == 0) return;

            Vector3 basePosUnity = KinematicOmniDrive.SharedRobotPosition;

            // Convert Unity robot pose to ROS map frame (x_map = z_unity, y_map = -x_unity, yaw_map = SharedRobotYawRad)
            float robotX = basePosUnity.z;
            float robotY = -basePosUnity.x;
            float robotYaw = KinematicOmniDrive.SharedRobotYawRad;
            float cosY = Mathf.Cos(robotYaw);
            float sinY = Mathf.Sin(robotYaw);

            // Sensor origin in ROS map frame (livox_tf: x=0.24, y=-0.25, z=1.09)
            float sensorOx = robotX + cosY * 0.24f - sinY * (-0.25f);
            float sensorOy = robotY + sinY * 0.24f + cosY * (-0.25f);
            float sensorOz = 1.09f;

            int count = pointsPerScan;
            int startIndex = _scanStartIndex;
            _scanStartIndex = (_scanStartIndex + count) % _totalRays;

            float[] hitDistances = new float[count];
            float minR = minRange;
            float maxR = maxRange;

            Parallel.For(0, count, i =>
            {
                int rayIdx = (startIndex + i) % _totalRays;
                Vector3 dBase = _rayDirBaseRos[rayIdx];

                // Reject self-hits on robot chassis (radius < 0.65m and z_base in [0.05, 1.25])
                float dxMap = cosY * dBase.x - sinY * dBase.y;
                float dyMap = sinY * dBase.x + cosY * dBase.y;
                float dzMap = dBase.z;

                float t = RaycastFieldInMap(sensorOx, sensorOy, sensorOz, dxMap, dyMap, dzMap, minR, maxR);
                if (t > 0f)
                {
                    float xb = 0.24f + t * dBase.x;
                    float yb = -0.25f + t * dBase.y;
                    float zb = 1.09f + t * dBase.z;
                    if (xb * xb + yb * yb < 0.65f * 0.65f && zb > 0.05f && zb < 1.25f)
                    {
                        t = -1.0f;
                    }
                }
                hitDistances[i] = t;
            });

            double startSimTimeSec = endSimTimeSec - scanPeriodSec;
            int headerSec = (int)startSimTimeSec;
            uint headerNanosec = (uint)((startSimTimeSec - headerSec) * 1e9);

            int pointStep = 24;
            byte[] pointData = new byte[count * pointStep];
            int validPoints = 0;

            for (int i = 0; i < count; i++)
            {
                float dist = hitDistances[i];
                if (dist <= 0f) continue;

                int rayIdx = (startIndex + i) % _totalRays;
                Vector3 dBase = _rayDirBaseRos[rayIdx];
                float zb = 1.09f + dist * dBase.z;
                if (zb <= 0.02f && (i & 3) != 0) continue;

                Vector3 ptLivox = _rayDirLivoxRos[rayIdx] * dist;
                float intensity = Mathf.Clamp01(1.0f - dist / maxR) * 255.0f;
                double ptTimeSec = startSimTimeSec + ((double)i / Math.Max(1, count - 1)) * scanPeriodSec;
                double stampNs = ptTimeSec * 1e9;

                int offset = validPoints * pointStep;
                Buffer.BlockCopy(BitConverter.GetBytes(ptLivox.x), 0, pointData, offset + 0, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(ptLivox.y), 0, pointData, offset + 4, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(ptLivox.z), 0, pointData, offset + 8, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(intensity), 0, pointData, offset + 12, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(stampNs), 0, pointData, offset + 16, 8);

                validPoints++;
                if (validPoints >= 5500) break;
            }

            if (validPoints == 0) return;

            byte[] trimmedData = new byte[validPoints * pointStep];
            Buffer.BlockCopy(pointData, 0, trimmedData, 0, trimmedData.Length);

            sensor_msgs.msg.PointCloud2 cloud = new sensor_msgs.msg.PointCloud2();
            cloud.Header.Frame_id = frameId;
            cloud.Header.Stamp.Sec = headerSec;
            cloud.Header.Stamp.Nanosec = headerNanosec;
            cloud.Height = 1;
            cloud.Width = (uint)validPoints;
            cloud.Is_bigendian = false;
            cloud.Point_step = (uint)pointStep;
            cloud.Row_step = (uint)(validPoints * pointStep);
            cloud.Is_dense = true;
            cloud.Data = trimmedData;

            cloud.Fields = new sensor_msgs.msg.PointField[5];
            cloud.Fields[0] = new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 };
            cloud.Fields[1] = new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 };
            cloud.Fields[2] = new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 };
            cloud.Fields[3] = new sensor_msgs.msg.PointField { Name = "intensity", Offset = 12, Datatype = 7, Count = 1 };
            cloud.Fields[4] = new sensor_msgs.msg.PointField { Name = "timestamp", Offset = 16, Datatype = 8, Count = 1 };

            _cloudPub.Publish(cloud);
        }
    }
}
