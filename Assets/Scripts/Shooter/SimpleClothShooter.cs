using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using RobotSim.Environment;

namespace RobotSim.Shooter
{
    public class SimpleClothShooter : MonoBehaviour
    {
        [Header("Shooter Specs")]
        [SerializeField] private float launchAngleDeg = 22.0f;
        [SerializeField] private GameObject clothPrefab;
        [SerializeField] private float defaultSpeedRatio = 0.4f;

        public static bool IsClothFlying
        {
            get { lock (ClothLock) { return _clothFlying; } }
        }

        public static Vector3 ActiveClothWorldPos
        {
            get { lock (ClothLock) { return _clothWorldPos; } }
        }

        private static readonly object ClothLock = new object();
        private static bool _clothFlying = false;
        private static Vector3 _clothWorldPos = Vector3.zero;
        private static Vector3 _clothWorldVel = Vector3.zero;
        private static float _clothFlightElapsed = 0f;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private ISubscription<std_msgs.msg.Bool> _subThrow;
        private ISubscription<std_msgs.msg.Float32> _subSpeedRatio;
        private ISubscription<geometry_msgs.msg.Twist> _subReleaseTwist;
        private ISubscription<std_msgs.msg.Bool> _subResetCloth;

        private float _currentSpeedRatio = 0.5f;
        private bool _lastThrowState = false;
        private bool _pendingVisualSpawn = false;
        private bool _pendingReset = false;

        private Mesh _cachedClothMesh;
        private Material[] _cachedClothMats;
        private GameObject _activeVisualCloth;
        private readonly List<GameObject> _spawnedCloths = new List<GameObject>();

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
            EnsureClothMeshLoaded();
            InitROS2();
        }

        private void EnsureClothMeshLoaded()
        {
            if (_cachedClothMesh != null) return;
            IsaacSceneMeta meta = FieldMapBuilder.LoadSceneMeta(out byte[] binData);
            if (meta != null && binData != null && meta.meshes != null)
            {
                foreach (var entry in meta.meshes)
                {
                    if (entry.category == "cloth")
                    {
                        _cachedClothMesh = FieldMapBuilder.BuildUnityMesh(entry, binData, out _cachedClothMats);
                        break;
                    }
                }
            }
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                _node = _ros2Unity.CreateNode("cloth_shooter_sim");
                _subThrow = _node.CreateSubscription<std_msgs.msg.Bool>("/belt/throw", OnThrowMsg);
                _subSpeedRatio = _node.CreateSubscription<std_msgs.msg.Float32>("/belt/speed_ratio", msg => _currentSpeedRatio = Mathf.Clamp01(msg.Data));
                _subReleaseTwist = _node.CreateSubscription<geometry_msgs.msg.Twist>("/launcher/release_twist", OnReleaseTwistMsg);
                _subResetCloth = _node.CreateSubscription<std_msgs.msg.Bool>("/reset_cloth", msg => { if (msg.Data) TriggerReset(); });
                Debug.Log("[SimpleClothShooter] Subscribed to /belt/throw, /belt/speed_ratio, /launcher/release_twist, /reset_cloth");
            }
        }

        private void TriggerReset()
        {
            lock (ClothLock)
            {
                _clothFlying = false;
                _clothFlightElapsed = 0f;
            }
            _pendingReset = true;
        }

        private void StartBallisticFlight(float speedRatio, Vector3? customLocalVel)
        {
            Vector3 robotPos = KinematicOmniDrive.SharedRobotPosition;
            Quaternion robotRot = KinematicOmniDrive.SharedRobotRotation;
            // Spawn inside realsense_dust_cloth launch gate (x_rob=0.90m, y_rob=0.0m, z_rob=1.52m)
            Vector3 spawnPos = robotPos + robotRot * new Vector3(0f, 1.52f, 0.90f);

            Vector3 worldVel;
            if (customLocalVel.HasValue && customLocalVel.Value.sqrMagnitude > 1e-4f)
            {
                worldVel = robotRot * customLocalVel.Value;
            }
            else
            {
                float releaseVelocity = Mathf.Clamp(GetReleaseVelocityFromRatio(speedRatio), 3.0f, 6.5f);
                float pitchRad = launchAngleDeg * Mathf.Deg2Rad;
                Vector3 localDir = new Vector3(0f, Mathf.Sin(pitchRad), Mathf.Cos(pitchRad)).normalized;
                worldVel = (robotRot * localDir) * releaseVelocity;
            }

            lock (ClothLock)
            {
                _clothWorldPos = spawnPos;
                _clothWorldVel = worldVel;
                _clothFlightElapsed = 0f;
                _clothFlying = true;
            }
            _pendingVisualSpawn = true;
        }

        public static void StepClothPhysics(float dt)
        {
            lock (ClothLock)
            {
                if (!_clothFlying) return;
                _clothFlightElapsed += dt;
                Vector3 drag = -0.35f * _clothWorldVel;
                _clothWorldVel += (new Vector3(0f, -9.81f, 0f) + drag) * dt;
                _clothWorldPos += _clothWorldVel * dt;

                if (_clothWorldPos.y <= 0.03f || _clothFlightElapsed > 5.0f)
                {
                    _clothWorldPos.y = 0.03f;
                    _clothFlying = false;
                }
            }
        }

        private void OnThrowMsg(std_msgs.msg.Bool msg)
        {
            if (msg.Data && !_lastThrowState)
            {
                StartBallisticFlight(_currentSpeedRatio, null);
            }
            _lastThrowState = msg.Data;
        }

        private void OnReleaseTwistMsg(geometry_msgs.msg.Twist msg)
        {
            Vector3 unityLocalVel = new Vector3(-(float)msg.Linear.Y, (float)msg.Linear.Z, (float)msg.Linear.X);
            StartBallisticFlight(_currentSpeedRatio, unityLocalVel.sqrMagnitude > 1e-4f ? unityLocalVel : (Vector3?)null);
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

            if (Input.GetKeyDown(KeyCode.Space))
            {
                ShootCloth(_currentSpeedRatio, null);
            }

            if (_pendingVisualSpawn)
            {
                _pendingVisualSpawn = false;
                SpawnVisualCloth();
            }

            if (_activeVisualCloth != null)
            {
                lock (ClothLock)
                {
                    _activeVisualCloth.transform.position = _clothWorldPos;
                }
            }
        }

        public void ResetCloths()
        {
            for (int i = _spawnedCloths.Count - 1; i >= 0; i--)
            {
                if (_spawnedCloths[i] != null)
                {
                    Destroy(_spawnedCloths[i]);
                }
            }
            _spawnedCloths.Clear();
            _activeVisualCloth = null;
        }

        public void ShootCloth(float speedRatio, Vector3? customLocalVel = null)
        {
            StartBallisticFlight(speedRatio, customLocalVel);
        }

        private void SpawnVisualCloth()
        {
            EnsureClothMeshLoaded();
            Vector3 spawnPos;
            lock (ClothLock)
            {
                spawnPos = _clothWorldPos;
            }

            GameObject clothObj;
            if (clothPrefab != null)
            {
                clothObj = Instantiate(clothPrefab, spawnPos, transform.rotation);
            }
            else if (_cachedClothMesh != null)
            {
                clothObj = new GameObject("marker_cloth");
                clothObj.transform.position = spawnPos;
                clothObj.transform.rotation = transform.rotation;

                MeshFilter mf = clothObj.AddComponent<MeshFilter>();
                mf.sharedMesh = _cachedClothMesh;

                MeshRenderer mr = clothObj.AddComponent<MeshRenderer>();
                mr.sharedMaterials = _cachedClothMats;
            }
            else
            {
                clothObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                clothObj.name = "SimulatedDustCloth";
                clothObj.transform.position = spawnPos;
                clothObj.transform.rotation = transform.rotation;
                clothObj.transform.localScale = new Vector3(0.25f, 0.03f, 0.25f);
                clothObj.GetComponent<Renderer>().material.color = Color.white;
            }

            _activeVisualCloth = clothObj;
            _spawnedCloths.Add(clothObj);
            Destroy(clothObj, 15.0f);
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
