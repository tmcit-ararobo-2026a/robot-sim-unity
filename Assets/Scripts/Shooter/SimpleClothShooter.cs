using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using RobotSim.Control;

namespace RobotSim.Shooter
{
    public class SimpleClothShooter : MonoBehaviour
    {
        [Header("Shooter Specs")]
        [SerializeField] private float launchAngleDeg = 22.0f;
        [SerializeField] private GameObject clothPrefab;
        [SerializeField] private float defaultSpeedRatio = 0.4f;

        [Header("Ballistic Model (gn10-navigation compatible)")]
        [Tooltip("Ballistic conversion scale: v_belt = scale * v0 + offset")]
        [SerializeField] private float velocityScale = 0.6806f;
        [SerializeField] private float velocityOffset = -0.0726f;
        [SerializeField] private float minReleaseVelocity = 2.0f;
        [SerializeField] private float maxReleaseVelocity = 14.0f;

        public static bool IsClothFlying
        {
            get
            {
                lock (ClothLock)
                {
                    for (int i = 0; i < _activeFlights.Count; i++)
                    {
                        if (_activeFlights[i].isFlying) return true;
                    }
                    return false;
                }
            }
        }

        public static Vector3 ActiveClothWorldPos
        {
            get
            {
                lock (ClothLock)
                {
                    for (int i = _activeFlights.Count - 1; i >= 0; i--)
                    {
                        if (_activeFlights[i].isFlying) return _activeFlights[i].pos;
                    }
                    return _lastClothWorldPos;
                }
            }
        }

        public static List<Vector3> GetAllFlyingClothPositions()
        {
            lock (ClothLock)
            {
                var list = new List<Vector3>(_activeFlights.Count);
                for (int i = 0; i < _activeFlights.Count; i++)
                {
                    if (_activeFlights[i].isFlying)
                    {
                        list.Add(_activeFlights[i].pos);
                    }
                }
                return list;
            }
        }

        private class ClothFlight
        {
            public int id;
            public SimpleClothShooter owner;
            public Rigidbody body;
            public Vector3 pos;
            public Vector3 vel;
            public Quaternion rot;
            public float flightElapsed;
            public bool isFlying;
            public float groundTime;
        }

        private static readonly object ClothLock = new object();
        private static readonly List<ClothFlight> _activeFlights = new List<ClothFlight>();
        private static int _nextClothId = 0;
        private static Vector3 _lastClothWorldPos = Vector3.zero;

        private readonly Dictionary<int, GameObject> _visualClothMap = new Dictionary<int, GameObject>();

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;

        // Subscriptions
        private ISubscription<std_msgs.msg.Bool> _subThrow;
        private ISubscription<std_msgs.msg.Float32> _subSpeedRatio;
        private ISubscription<geometry_msgs.msg.Twist> _subReleaseTwist;
        private ISubscription<std_msgs.msg.Bool> _subResetCloth;
        private ISubscription<std_msgs.msg.Float32> _subBeltLauncherSpeedOp;
        private ISubscription<std_msgs.msg.Float32> _subBeltLauncherSpeedRobot;

        // Publishers for ROS 2 / mainboard telemetry parity
        private IPublisher<std_msgs.msg.UInt8> _pubShotCount;
        private IPublisher<std_msgs.msg.Float32> _pubBeltVelocity;
        private IPublisher<std_msgs.msg.Float32> _pubBeltTarget;

        private float _currentSpeedRatio = 0.5f;
        private float _currentBeltLauncherSpeed = 0f;
        private bool _hasBeltLauncherSpeed = false;
        private byte _shotCount = 0;
        private bool _lastThrowState = false;
        private volatile bool _pendingReset = false;

        public float CurrentBeltLauncherSpeed => _currentBeltLauncherSpeed;
        public byte ShotCount => _shotCount;

        private static readonly (float level, float speed_ratio, float release_velocity, float flight_distance)[] MEASURED_LUT = new[]
        {
            (1f,  0.1f, 1.550000000f, 0.000000000f),
            (2f,  0.2f, 2.296666667f, 0.533333333f),
            (3f,  0.3f, 4.763333333f, 2.666666667f),
            (4f,  0.4f, 5.953333333f, 3.966666667f),
            (5f,  0.5f, 8.840000000f, 3.566666667f),
            (6f,  0.6f, 11.23333333f, 4.100000000f),
            (7f,  0.7f, 8.516666667f, 3.333333333f),
            (8f,  0.8f, 9.040000000f, 3.566666667f),
            (9f,  0.9f, 9.420000000f, 3.833333333f),
            (10f, 1.0f, 10.84666667f, 3.966666667f)
        };

        private void Start()
        {
            _currentSpeedRatio = defaultSpeedRatio;
            if (clothPrefab == null || clothPrefab.GetComponent<Rigidbody>() == null)
            {
                Debug.LogError("[SimpleClothShooter] Assign a cloth prefab with a Rigidbody.", this);
                enabled = false;
                return;
            }
            InitROS2();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                _node = _ros2Unity.CreateNode("cloth_shooter_sim");

                // Legacy subscriptions
                _subThrow = _node.CreateSubscription<std_msgs.msg.Bool>("/belt/throw", OnThrowMsg);
                _subSpeedRatio = _node.CreateSubscription<std_msgs.msg.Float32>("/belt/speed_ratio", msg => _currentSpeedRatio = Mathf.Clamp01(msg.Data));
                _subReleaseTwist = _node.CreateSubscription<geometry_msgs.msg.Twist>("/launcher/release_twist", OnReleaseTwistMsg);
                _subResetCloth = _node.CreateSubscription<std_msgs.msg.Bool>("/reset_cloth", msg => { if (msg.Data) TriggerReset(); });

                // gn10-navigation belt launcher speed subscriptions
                _subBeltLauncherSpeedOp = _node.CreateSubscription<std_msgs.msg.Float32>(
                    "/robot/operation/belt_launcher_speed", OnBeltLauncherSpeedMsg);
                _subBeltLauncherSpeedRobot = _node.CreateSubscription<std_msgs.msg.Float32>(
                    "/robot/belt_launcher_speed", OnBeltLauncherSpeedMsg);

                // Telemetry feedback publishers
                _pubShotCount = _node.CreatePublisher<std_msgs.msg.UInt8>("/robot/feedback/shot_count");
                _pubBeltVelocity = _node.CreatePublisher<std_msgs.msg.Float32>("/robot/feedback/belt_launcher/velocity");
                _pubBeltTarget = _node.CreatePublisher<std_msgs.msg.Float32>("/robot/feedback/belt_launcher/target");

                Debug.Log("[SimpleClothShooter] Subscribed to /robot/operation/belt_launcher_speed, /belt/throw, etc.");
            }
        }

        private void OnBeltLauncherSpeedMsg(std_msgs.msg.Float32 msg)
        {
            _currentBeltLauncherSpeed = Mathf.Max(0f, msg.Data);
            _hasBeltLauncherSpeed = true;
        }

        private void TriggerReset()
        {
            lock (ClothLock)
            {
                _activeFlights.Clear();
            }
            _pendingReset = true;
        }

        public float BeltSpeedToBallisticVelocity(float vBelt)
        {
            if (Mathf.Abs(velocityScale) < 1e-5f)
            {
                return Mathf.Clamp(vBelt, minReleaseVelocity, maxReleaseVelocity);
            }
            float v0 = (vBelt - velocityOffset) / velocityScale;
            return Mathf.Clamp(v0, minReleaseVelocity, maxReleaseVelocity);
        }

        private void StartBallisticFlight(float? customReleaseVelocity, float speedRatio, Vector3? customLocalVel)
        {
            var motion = KinematicOmniDrive.GetMotionState();
            Vector3 robotPos = motion.Position;
            Quaternion robotRot = motion.Rotation;
            // Spawn inside realsense_dust_cloth launch gate (x_rob=0.90m, y_rob=0.0m, z_rob=1.52m)
            Vector3 spawnPos = robotPos + robotRot * new Vector3(0f, 1.52f, 0.90f);

            Vector3 worldVel;
            float releaseVel;

            if (customLocalVel.HasValue && customLocalVel.Value.sqrMagnitude > 1e-4f)
            {
                worldVel = robotRot * customLocalVel.Value;
                releaseVel = customLocalVel.Value.magnitude;
            }
            else
            {
                if (customReleaseVelocity.HasValue)
                {
                    releaseVel = Mathf.Clamp(customReleaseVelocity.Value, minReleaseVelocity, maxReleaseVelocity);
                }
                else if (_hasBeltLauncherSpeed && _currentBeltLauncherSpeed > 0.1f)
                {
                    releaseVel = BeltSpeedToBallisticVelocity(_currentBeltLauncherSpeed);
                }
                else
                {
                    releaseVel = Mathf.Clamp(GetReleaseVelocityFromRatio(speedRatio), 3.0f, 6.5f);
                }

                float pitchRad = launchAngleDeg * Mathf.Deg2Rad;
                Vector3 localDir = new Vector3(0f, Mathf.Sin(pitchRad), Mathf.Cos(pitchRad)).normalized;
                worldVel = (robotRot * localDir) * releaseVel;
            }

            lock (ClothLock)
            {
                int newId = ++_nextClothId;
                _activeFlights.Add(new ClothFlight
                {
                    id = newId,
                    owner = this,
                    pos = spawnPos,
                    vel = worldVel,
                    rot = robotRot,
                    flightElapsed = 0f,
                    isFlying = true,
                    groundTime = 0f
                });
                _lastClothWorldPos = spawnPos;
            }

            // Publish feedback telemetry
            _shotCount++;
            PublishFeedbackTelemetry(releaseVel);
        }

        private void PublishFeedbackTelemetry(float releaseVel)
        {
            if (_node == null) return;

            if (_pubShotCount != null)
            {
                _pubShotCount.Publish(new std_msgs.msg.UInt8 { Data = _shotCount });
            }
            if (_pubBeltVelocity != null)
            {
                float reportedBeltSpeed = (_currentBeltLauncherSpeed > 0.05f)
                    ? _currentBeltLauncherSpeed
                    : (velocityScale * releaseVel + velocityOffset);
                _pubBeltVelocity.Publish(new std_msgs.msg.Float32 { Data = reportedBeltSpeed });
            }
            if (_pubBeltTarget != null)
            {
                _pubBeltTarget.Publish(new std_msgs.msg.Float32 { Data = _currentBeltLauncherSpeed });
            }
        }

        private void OnThrowMsg(std_msgs.msg.Bool msg)
        {
            if (msg.Data && !_lastThrowState)
            {
                StartBallisticFlight(null, _currentSpeedRatio, null);
            }
            _lastThrowState = msg.Data;
        }

        private void OnReleaseTwistMsg(geometry_msgs.msg.Twist msg)
        {
            Vector3 unityLocalVel = new Vector3(-(float)msg.Linear.Y, (float)msg.Linear.Z, (float)msg.Linear.X);
            StartBallisticFlight(null, _currentSpeedRatio, unityLocalVel.sqrMagnitude > 1e-4f ? unityLocalVel : (Vector3?)null);
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }

            if (_pendingReset)
            {
                _pendingReset = false;
                ResetCloths();
            }

            // Space key triggers shoot only if Dashboard is not active or as fallback
            if (Input.GetKeyDown(KeyCode.Space))
            {
                ShootCloth(_currentSpeedRatio, null);
            }

            SyncClothVisuals();
        }

        private void SyncClothVisuals()
        {
            // ROS callbacks enqueue launches. Unity objects and physics are handled
            // only on the main thread; sensors read the locked physical snapshot.
            lock (ClothLock)
            {
                for (int i = _activeFlights.Count - 1; i >= 0; i--)
                {
                    var flight = _activeFlights[i];
                    if (!ReferenceEquals(flight.owner, this)) continue;

                    if (!_visualClothMap.TryGetValue(flight.id, out GameObject cloth))
                    {
                        cloth = Instantiate(clothPrefab, flight.pos, flight.rot);
                        _visualClothMap.Add(flight.id, cloth);
                        flight.body = cloth.GetComponent<Rigidbody>();
                        flight.body.linearVelocity = flight.vel;
                    }
                    if (cloth == null)
                    {
                        _visualClothMap.Remove(flight.id);
                        _activeFlights.RemoveAt(i);
                        continue;
                    }

                    flight.pos = flight.body.position;
                    flight.vel = flight.body.linearVelocity;
                    flight.rot = flight.body.rotation;
                    flight.isFlying = !flight.body.IsSleeping();
                    flight.flightElapsed += Time.deltaTime;
                    flight.groundTime = flight.isFlying ? 0f : flight.groundTime + Time.deltaTime;
                    _lastClothWorldPos = flight.pos;

                    if (flight.groundTime > 15f || flight.flightElapsed > 25f)
                    {
                        Destroy(cloth);
                        _visualClothMap.Remove(flight.id);
                        _activeFlights.RemoveAt(i);
                    }
                }
            }
        }

        private void OnDestroy()
        {
            lock (ClothLock)
                _activeFlights.RemoveAll(flight => ReferenceEquals(flight.owner, this));
            foreach (var cloth in _visualClothMap.Values)
                if (cloth != null) Destroy(cloth);
            _visualClothMap.Clear();
        }

        public void ResetCloths()
        {
            lock (ClothLock)
            {
                _activeFlights.Clear();
            }
            foreach (var kvp in _visualClothMap)
            {
                if (kvp.Value != null)
                {
                    Destroy(kvp.Value);
                }
            }
            _visualClothMap.Clear();
        }

        public void ShootCloth(float speedRatio, Vector3? customLocalVel = null)
        {
            StartBallisticFlight(null, speedRatio, customLocalVel);
        }

        public void ShootClothWithBeltSpeed(float beltSpeedMs)
        {
            float v0 = BeltSpeedToBallisticVelocity(beltSpeedMs);
            StartBallisticFlight(v0, _currentSpeedRatio, null);
        }

        public void ShootClothWithBallisticVelocity(float v0)
        {
            StartBallisticFlight(v0, _currentSpeedRatio, null);
        }

        private static float GetReleaseVelocityFromRatio(float ratio)
        {
            float level = Mathf.Clamp(ratio * 10.0f, 1.0f, 10.0f);
            if (level <= MEASURED_LUT[0].level) return MEASURED_LUT[0].release_velocity;
            if (level >= MEASURED_LUT[MEASURED_LUT.Length - 1].level) return MEASURED_LUT[MEASURED_LUT.Length - 1].release_velocity;

            for (int i = 0; i < MEASURED_LUT.Length - 1; i++)
            {
                if (level >= MEASURED_LUT[i].level && level <= MEASURED_LUT[i + 1].level)
                {
                    float t = (level - MEASURED_LUT[i].level) / (MEASURED_LUT[i + 1].level - MEASURED_LUT[i].level);
                    return Mathf.Lerp(MEASURED_LUT[i].release_velocity, MEASURED_LUT[i + 1].release_velocity, t);
                }
            }
            return 8.84f;
        }
    }
}
