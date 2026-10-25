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
            public Vector3 pos;
            public Vector3 vel;
            public Quaternion rot;
            public float flightElapsed;
            public bool isFlying;
            public float groundTime;
        }

        private struct ClothStateSnapshot
        {
            public int id;
            public Vector3 pos;
            public Quaternion rot;
            public bool isFlying;
        }

        private static readonly object ClothLock = new object();
        private static readonly List<ClothFlight> _activeFlights = new List<ClothFlight>();
        private static int _nextClothId = 0;
        private static Vector3 _lastClothWorldPos = Vector3.zero;

        private readonly Dictionary<int, GameObject> _visualClothMap = new Dictionary<int, GameObject>();

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private ISubscription<std_msgs.msg.Bool> _subThrow;
        private ISubscription<std_msgs.msg.Float32> _subSpeedRatio;
        private ISubscription<geometry_msgs.msg.Twist> _subReleaseTwist;
        private ISubscription<std_msgs.msg.Bool> _subResetCloth;

        private float _currentSpeedRatio = 0.5f;
        private bool _lastThrowState = false;
        private bool _pendingReset = false;

        private Mesh _cachedClothMesh;
        private Material[] _cachedClothMats;

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
                _activeFlights.Clear();
            }
            _pendingReset = true;
        }

        private void StartBallisticFlight(float speedRatio, Vector3? customLocalVel)
        {
            var motion = KinematicOmniDrive.GetMotionState();
            Vector3 robotPos = motion.Position;
            Quaternion robotRot = motion.Rotation;
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
                int newId = ++_nextClothId;
                _activeFlights.Add(new ClothFlight
                {
                    id = newId,
                    pos = spawnPos,
                    vel = worldVel,
                    rot = robotRot,
                    flightElapsed = 0f,
                    isFlying = true,
                    groundTime = 0f
                });
                _lastClothWorldPos = spawnPos;
            }
        }

        public static void StepClothPhysics(float dt)
        {
            if (dt <= 0f) return;

            lock (ClothLock)
            {
                for (int i = 0; i < _activeFlights.Count; i++)
                {
                    var f = _activeFlights[i];
                    if (!f.isFlying) continue;

                    f.flightElapsed += dt;
                    Vector3 drag = -0.35f * f.vel;
                    f.vel += (new Vector3(0f, -9.81f, 0f) + drag) * dt;
                    f.pos += f.vel * dt;

                    if (f.pos.y <= 0.03f || f.flightElapsed > 5.0f)
                    {
                        f.pos.y = 0.03f;
                        f.vel = Vector3.zero;
                        f.isFlying = false;
                    }
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

            SyncClothVisuals();
        }

        private void SyncClothVisuals()
        {
            float dt = Time.deltaTime;
            ClothStateSnapshot[] snapshots = null;
            List<int> toRemoveIds = null;

            lock (ClothLock)
            {
                if (_activeFlights.Count > 0)
                {
                    snapshots = new ClothStateSnapshot[_activeFlights.Count];
                    for (int i = 0; i < _activeFlights.Count; i++)
                    {
                        var f = _activeFlights[i];
                        if (!f.isFlying)
                        {
                            f.groundTime += dt;
                            if (f.groundTime > 15.0f || f.flightElapsed > 25.0f)
                            {
                                if (toRemoveIds == null) toRemoveIds = new List<int>();
                                toRemoveIds.Add(f.id);
                            }
                        }

                        snapshots[i] = new ClothStateSnapshot
                        {
                            id = f.id,
                            pos = f.pos,
                            rot = f.rot,
                            isFlying = f.isFlying
                        };
                    }

                    if (toRemoveIds != null)
                    {
                        for (int r = 0; r < toRemoveIds.Count; r++)
                        {
                            int remId = toRemoveIds[r];
                            for (int i = _activeFlights.Count - 1; i >= 0; i--)
                            {
                                if (_activeFlights[i].id == remId)
                                {
                                    _activeFlights.RemoveAt(i);
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            if (snapshots != null)
            {
                for (int i = 0; i < snapshots.Length; i++)
                {
                    int id = snapshots[i].id;
                    Vector3 pos = snapshots[i].pos;
                    Quaternion rot = snapshots[i].rot;

                    if (!_visualClothMap.TryGetValue(id, out GameObject visualObj) || visualObj == null)
                    {
                        visualObj = CreateVisualClothObject(pos, rot);
                        _visualClothMap[id] = visualObj;
                    }

                    if (visualObj != null)
                    {
                        visualObj.transform.position = pos;
                    }
                }
            }

            if (toRemoveIds != null)
            {
                for (int r = 0; r < toRemoveIds.Count; r++)
                {
                    int remId = toRemoveIds[r];
                    if (_visualClothMap.TryGetValue(remId, out GameObject obj))
                    {
                        if (obj != null) Destroy(obj);
                        _visualClothMap.Remove(remId);
                    }
                }
            }
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
            StartBallisticFlight(speedRatio, customLocalVel);
        }

        private GameObject CreateVisualClothObject(Vector3 spawnPos, Quaternion rot)
        {
            EnsureClothMeshLoaded();
            GameObject clothObj;
            if (clothPrefab != null)
            {
                clothObj = Instantiate(clothPrefab, spawnPos, rot);
            }
            else if (_cachedClothMesh != null)
            {
                clothObj = new GameObject("marker_cloth");
                clothObj.transform.position = spawnPos;
                clothObj.transform.rotation = rot;

                MeshFilter mf = clothObj.AddComponent<MeshFilter>();
                mf.sharedMesh = _cachedClothMesh;

                MeshRenderer mr = clothObj.AddComponent<MeshRenderer>();
                mr.sharedMaterials = _cachedClothMats;

                BoxCollider bc = clothObj.AddComponent<BoxCollider>();
                bc.isTrigger = true;
                bc.size = new Vector3(0.30f, 0.05f, 0.20f);
            }
            else
            {
                clothObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                clothObj.name = "SimulatedDustCloth";
                clothObj.transform.position = spawnPos;
                clothObj.transform.rotation = rot;
                clothObj.transform.localScale = new Vector3(0.25f, 0.03f, 0.25f);
                clothObj.GetComponent<Renderer>().material.color = Color.white;
            }

            Destroy(clothObj, 25.0f);
            return clothObj;
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
