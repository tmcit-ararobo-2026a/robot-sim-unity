using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace RobotSim.Opponent
{
    public enum OpponentState
    {
        Patrol,
        AimingAndThrowing,
        HeadingToReload,
        Reloading
    }

    [Serializable]
    public struct ObstacleRect2D
    {
        public string name;
        public float minX;
        public float maxX;
        public float minZ;
        public float maxZ;

        public ObstacleRect2D(string name, float minX, float maxX, float minZ, float maxZ)
        {
            this.name = name;
            this.minX = minX;
            this.maxX = maxX;
            this.minZ = minZ;
            this.maxZ = maxZ;
        }
    }

    [RequireComponent(typeof(Rigidbody))]
    public class OpponentRobotController : MonoBehaviour
    {
        [Header("Omni-Drive Kinematics")]
        [SerializeField] private float maxMoveSpeed = 1.0f;          // 最高巡回速度 (m/s)
        [SerializeField] private float reloadApproachSpeed = 0.75f;  // リロード帰還速度 (m/s)
        [SerializeField] private float acceleration = 2.5f;          // 加速度 (m/s^2)
        [SerializeField] private float rotateSpeedDeg = 120.0f;      // 旋回速度 (deg/s)
        [SerializeField] private float robotBoundingRadius = 0.52f;  // 機体保護半径 (m)
        [SerializeField] private float waypointTolerance = 0.18f;    // ノード到達判定 (m)

        [Header("Match FSM Timing")]
        [SerializeField] private float patrolDuration = 22.0f;       // 巡回・投擲フェーズ時間 (秒)
        [SerializeField] private float aimPauseDuration = 1.8f;      // 各投擲ポイントでの照準・射撃滞在時間 (秒)
        [SerializeField] private float reloadWaitDuration = 4.0f;    // リロードゾーン停止時間 (秒)

        [Header("Blue Zone Boundaries (World Space: X < -0.30 is Blue Zone)")]
        [SerializeField] private float blueZoneMinX = -5.70f;        // 青バックフェンス内面
        [SerializeField] private float blueZoneMaxX = -0.30f;        // 中央教壇の青側エッジ
        [SerializeField] private float blueZoneMinZ = -5.25f;        // 補充机側フェンス内面
        [SerializeField] private float blueZoneMaxZ = 5.25f;         // 奥側フェンス内面

        [Header("Reload Zone (Blue Start Zone next to desk_replenish_blue)")]
        // desk_replenish_blue World Center = (-1.06, 0.34, -4.77)
        // 青スタートゾーン内待機座標
        [SerializeField] private Vector3 reloadPosition = new Vector3(-2.00f, 0.00f, -4.55f);
        [SerializeField] private float reloadFacingYawDeg = 90.0f;   // 補充机 (+X方向) を向く

        [Header("Safe Corridor Waypoints (Blue Zone World Space)")]
        [SerializeField] private List<Vector3> corridorNodes = new List<Vector3>
        {
            new Vector3(-2.00f, 0.00f, -4.55f), // 0: Reload Zone (Blue Start Zone)
            new Vector3(-2.50f, 0.00f, -3.30f), // 1: Exit Reload / Left Entry
            new Vector3(-2.50f, 0.00f, -1.20f), // 2: Front-Left Throw Spot
            new Vector3(-1.92f, 0.00f,  0.55f), // 3: Front-Center Throw Spot (between bucket1 & flag)
            new Vector3(-2.45f, 0.00f,  2.10f), // 4: Front-Right Throw Spot
            new Vector3(-2.45f, 0.00f,  4.10f), // 5: Far-Right Throw Spot
            new Vector3(-3.95f, 0.00f,  1.90f), // 6: Back-Right Throw Spot
            new Vector3(-3.95f, 0.00f, -0.60f)  // 7: Back-Center Throw Spot
        };

        [Header("ROS 2 Teleop Override (Optional)")]
        [SerializeField] private bool enableRosTeleop = false;
        [SerializeField] private string cmdVelTopic = "/opponent/cmd_vel";

        public OpponentState CurrentState { get; private set; } = OpponentState.Patrol;
        public Vector3 CurrentVelocity { get; private set; } = Vector3.zero;
        public int CurrentTargetNode { get; private set; } = 1;

        private Rigidbody _rb;
        private float _phaseTimer = 0f;
        private float _aimTimer = 0f;
        private float _targetYawDeg = 90.0f; // デフォルトで赤ゾーン (+X) 方向を向く
        private readonly Queue<int> _navQueue = new Queue<int>();
        private int _currentNodeIndex = 0;

        // 無向グラフの隣接リスト（全エッジが障害物クリアランス検証済み）
        private static readonly int[][] AdjacencyGraph = new int[][]
        {
            new[] { 1 },          // 0 (Reload) <-> 1
            new[] { 0, 2, 7 },    // 1 (ExitReload) <-> 0, 2, 7
            new[] { 1, 3, 7 },    // 2 (FrontLeft) <-> 1, 3, 7
            new[] { 2, 4 },       // 3 (FrontCenter) <-> 2, 4
            new[] { 3, 5, 6 },    // 4 (FrontRight) <-> 3, 5, 6
            new[] { 4 },          // 5 (FarRight) <-> 4
            new[] { 4, 7 },       // 6 (BackRight) <-> 4, 7
            new[] { 1, 2, 6 }     // 7 (BackCenter) <-> 1, 2, 6
        };

        // 青ゾーン内の全固定障害物 (Unity World Space 実測AABB)
        private static readonly ObstacleRect2D[] BlueZoneObstacles = new ObstacleRect2D[]
        {
            new ObstacleRect2D("bucket1_blue",         -1.01f, -0.73f,  0.41f,  0.69f),
            new ObstacleRect2D("bucket2_platform_blue",-1.63f, -1.33f, -1.41f, -1.11f),
            new ObstacleRect2D("bucket3_platform_blue",-1.63f, -1.33f,  2.58f,  2.88f),
            new ObstacleRect2D("flag_blue",            -3.22f, -2.83f,  0.35f,  1.15f),
            new ObstacleRect2D("desk1_blue",           -4.08f, -3.63f, -2.57f, -2.02f),
            new ObstacleRect2D("desk2_blue",           -4.08f, -3.63f,  3.12f,  3.67f),
            new ObstacleRect2D("chair_blue",           -5.22f, -4.73f,  0.32f,  0.81f),
            new ObstacleRect2D("desk_replenish_blue",  -1.29f, -0.84f, -5.05f, -4.50f),
            new ObstacleRect2D("desk_control_blue",    -5.65f, -5.20f, -5.19f, -4.64f)
        };

        private static readonly Vector3[] RedTargetLookPoints = new Vector3[]
        {
            new Vector3(1.48f, 0.76f, -1.27f), // bucket2_red
            new Vector3(3.03f, 1.53f,  0.75f), // flag_red
            new Vector3(1.49f, 0.43f,  2.72f), // bucket3_red
            new Vector3(2.50f, 1.50f,  0.00f)  // Red center
        };

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private ISubscription<geometry_msgs.msg.Twist> _subCmdVel;
        private Vector3 _rosCmdLinear = Vector3.zero;
        private float _rosCmdAngularZ = 0f;
        private double _lastRosCmdTime = -10.0;

        private void Awake()
        {
            _rb = GetComponent<Rigidbody>();
            _rb.useGravity = false;
            _rb.isKinematic = true;
            _rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }

        private void Start()
        {
            // 初期位置を最寄りノードに合わせる
            _currentNodeIndex = FindNearestNodeIndex(transform.position);
            PlanRouteToRandomThrowSpot();
            SetState(OpponentState.Patrol);
            InitROS2();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                _node = _ros2Unity.CreateNode("opponent_robot_node");
                _subCmdVel = _node.CreateSubscription<geometry_msgs.msg.Twist>(cmdVelTopic, msg =>
                {
                    _rosCmdLinear = new Vector3(-(float)msg.Linear.Y, 0f, (float)msg.Linear.X);
                    _rosCmdAngularZ = -(float)msg.Angular.Z;
                    _lastRosCmdTime = Time.timeAsDouble;
                });
            }
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;

            // ROS 2 手動テレロップオーバーライド
            if (enableRosTeleop && (Time.timeAsDouble - _lastRosCmdTime < 0.5))
            {
                Vector3 desiredVel = transform.rotation * _rosCmdLinear;
                Vector3 nextPos = ResolveCollisionsAndBounds(transform.position + desiredVel * dt);
                CurrentVelocity = (nextPos - transform.position) / Mathf.Max(dt, 1e-4f);
                _rb.MovePosition(nextPos);

                float yawDelta = _rosCmdAngularZ * Mathf.Rad2Deg * dt;
                _rb.MoveRotation(Quaternion.Euler(0f, transform.eulerAngles.y + yawDelta, 0f));
                return;
            }

            switch (CurrentState)
            {
                case OpponentState.Patrol:
                    _phaseTimer += dt;
                    UpdatePatrol(dt);
                    if (_phaseTimer >= patrolDuration)
                    {
                        PlanRouteToNode(0); // Node 0 = ReloadZone
                        SetState(OpponentState.HeadingToReload);
                    }
                    break;

                case OpponentState.AimingAndThrowing:
                    _phaseTimer += dt;
                    _aimTimer += dt;
                    UpdateAimingAndThrowing(dt);
                    if (_phaseTimer >= patrolDuration)
                    {
                        PlanRouteToNode(0);
                        SetState(OpponentState.HeadingToReload);
                    }
                    else if (_aimTimer >= aimPauseDuration)
                    {
                        PlanRouteToRandomThrowSpot();
                        SetState(OpponentState.Patrol);
                    }
                    break;

                case OpponentState.HeadingToReload:
                    UpdateHeadingToReload(dt);
                    break;

                case OpponentState.Reloading:
                    _aimTimer += dt;
                    UpdateReloading(dt);
                    if (_aimTimer >= reloadWaitDuration)
                    {
                        _phaseTimer = 0f;
                        PlanRouteToRandomThrowSpot();
                        SetState(OpponentState.Patrol);
                    }
                    break;
            }
        }

        private void SetState(OpponentState nextState)
        {
            CurrentState = nextState;
            _aimTimer = 0f;
        }

        private void UpdatePatrol(float dt)
        {
            if (_navQueue.Count == 0)
            {
                // 投擲スポットに到着 → 照準＆投擲フェーズへ
                Vector3 lookTarget = RedTargetLookPoints[UnityEngine.Random.Range(0, RedTargetLookPoints.Length)];
                Vector3 toRed = lookTarget - transform.position;
                toRed.y = 0f;
                if (toRed.sqrMagnitude > 0.01f)
                {
                    _targetYawDeg = Quaternion.LookRotation(toRed.normalized, Vector3.up).eulerAngles.y;
                }
                SetState(OpponentState.AimingAndThrowing);
                return;
            }

            int nextNode = _navQueue.Peek();
            CurrentTargetNode = nextNode;
            Vector3 targetPos = corridorNodes[nextNode];

            if (MoveHolonomicTowards(targetPos, maxMoveSpeed, _targetYawDeg, dt))
            {
                _currentNodeIndex = _navQueue.Dequeue();
            }
        }

        private void UpdateAimingAndThrowing(float dt)
        {
            // 投擲スポットで小刻みに左右オムニ回避運動をしながら赤陣地へ照準を合わせる
            Vector3 anchor = corridorNodes[_currentNodeIndex];
            float strafeOffset = Mathf.Sin(_aimTimer * 3.5f) * 0.14f;
            Vector3 strafeTarget = anchor + new Vector3(0f, 0f, strafeOffset);

            MoveHolonomicTowards(strafeTarget, 0.35f, _targetYawDeg, dt);
        }

        private void UpdateHeadingToReload(float dt)
        {
            if (_navQueue.Count == 0)
            {
                if (MoveHolonomicTowards(reloadPosition, reloadApproachSpeed, reloadFacingYawDeg, dt))
                {
                    _currentNodeIndex = 0;
                    CurrentVelocity = Vector3.zero;
                    SetState(OpponentState.Reloading);
                }
                return;
            }

            int nextNode = _navQueue.Peek();
            CurrentTargetNode = nextNode;
            Vector3 targetPos = (nextNode == 0) ? reloadPosition : corridorNodes[nextNode];

            if (MoveHolonomicTowards(targetPos, reloadApproachSpeed, reloadFacingYawDeg, dt))
            {
                _currentNodeIndex = _navQueue.Dequeue();
            }
        }

        private void UpdateReloading(float dt)
        {
            // スタートゾーン内で完全停止し、補充スポット机 (+X方向) へ正対
            MoveHolonomicTowards(reloadPosition, 0.3f, reloadFacingYawDeg, dt);
            if ((transform.position - reloadPosition).sqrMagnitude < 0.01f)
            {
                CurrentVelocity = Vector3.zero;
            }
        }

        private bool MoveHolonomicTowards(Vector3 targetPos, float speedLimit, float desiredYawDeg, float dt)
        {
            Vector3 currentPos = transform.position;
            Vector3 toTarget = targetPos - currentPos;
            toTarget.y = 0f;
            float dist = toTarget.magnitude;

            Vector3 desiredVel = Vector3.zero;
            if (dist > 0.01f)
            {
                // 目標接近時はスムーズに減速
                float rampedSpeed = Mathf.Min(speedLimit, dist * 2.5f);
                desiredVel = toTarget.normalized * rampedSpeed;

                // 障害物からの反発ポテンシャル場を加算して滑らかに回避
                Vector3 repulsion = ComputeObstacleRepulsion(currentPos);
                desiredVel += repulsion;
                if (desiredVel.magnitude > speedLimit)
                {
                    desiredVel = desiredVel.normalized * speedLimit;
                }
            }

            CurrentVelocity = Vector3.MoveTowards(CurrentVelocity, desiredVel, acceleration * dt);
            Vector3 candidatePos = currentPos + CurrentVelocity * dt;

            // 厳密な障害物押し戻し & 青ゾーン境界クランプ
            Vector3 safePos = ResolveCollisionsAndBounds(candidatePos);
            CurrentVelocity = (safePos - currentPos) / Mathf.Max(dt, 1e-4f);
            _rb.MovePosition(safePos);

            // オムニホイール独立姿勢制御（常に赤ゾーンまたは補充机に正対）
            Quaternion targetRot = Quaternion.Euler(0f, desiredYawDeg, 0f);
            _rb.MoveRotation(Quaternion.RotateTowards(transform.rotation, targetRot, rotateSpeedDeg * dt));

            return dist <= waypointTolerance;
        }

        private Vector3 ComputeObstacleRepulsion(Vector3 pos)
        {
            Vector3 totalRepulsion = Vector3.zero;
            float influenceRadius = robotBoundingRadius + 0.30f;

            for (int i = 0; i < BlueZoneObstacles.Length; i++)
            {
                ObstacleRect2D ob = BlueZoneObstacles[i];
                float cx = Mathf.Clamp(pos.x, ob.minX, ob.maxX);
                float cz = Mathf.Clamp(pos.z, ob.minZ, ob.maxZ);
                Vector3 diff = new Vector3(pos.x - cx, 0f, pos.z - cz);
                float d = diff.magnitude;

                if (d > 1e-4f && d < influenceRadius)
                {
                    float strength = (influenceRadius - d) / influenceRadius;
                    totalRepulsion += diff.normalized * (strength * 0.8f);
                }
            }
            return totalRepulsion;
        }

        private Vector3 ResolveCollisionsAndBounds(Vector3 pos)
        {
            float r = robotBoundingRadius;

            // 1. 青ゾーン外周フェンスおよび中央教壇 (X: [-5.70, -0.30], Z: [-5.25, +5.25]) から絶対にはみ出さないクランプ
            pos.x = Mathf.Clamp(pos.x, blueZoneMinX + r, blueZoneMaxX - r);
            pos.z = Mathf.Clamp(pos.z, blueZoneMinZ + r, blueZoneMaxZ - r);
            pos.y = 0.00f;

            // 2. 全固定障害物 (バケツ1/2/3、机1/2、補充机、コントロール机、旗、椅子) との厳密な Circle-vs-AABB 押し出し判定
            float safetyMargin = r + 0.05f;
            for (int iter = 0; iter < 2; iter++)
            {
                for (int i = 0; i < BlueZoneObstacles.Length; i++)
                {
                    ObstacleRect2D ob = BlueZoneObstacles[i];
                    float closestX = Mathf.Clamp(pos.x, ob.minX, ob.maxX);
                    float closestZ = Mathf.Clamp(pos.z, ob.minZ, ob.maxZ);

                    float dx = pos.x - closestX;
                    float dz = pos.z - closestZ;
                    float distSq = dx * dx + dz * dz;

                    if (distSq < safetyMargin * safetyMargin)
                    {
                        float dist = Mathf.Sqrt(distSq);
                        if (dist > 1e-4f)
                        {
                            float push = safetyMargin - dist;
                            pos.x += (dx / dist) * push;
                            pos.z += (dz / dist) * push;
                        }
                        else
                        {
                            // AABB内部に入り込んだ場合は最短脱出方向へ押し出す
                            float midX = (ob.minX + ob.maxX) * 0.5f;
                            pos.x = (pos.x < midX) ? (ob.minX - safetyMargin) : (ob.maxX + safetyMargin);
                        }
                    }
                }

                pos.x = Mathf.Clamp(pos.x, blueZoneMinX + r, blueZoneMaxX - r);
                pos.z = Mathf.Clamp(pos.z, blueZoneMinZ + r, blueZoneMaxZ - r);
            }

            return pos;
        }

        private void PlanRouteToRandomThrowSpot()
        {
            // Node 2..7 の投擲スポットから現在地以外のスポットを選択
            int nextGoal = UnityEngine.Random.Range(2, corridorNodes.Count);
            if (nextGoal == _currentNodeIndex)
            {
                nextGoal = ((nextGoal - 2 + 1) % (corridorNodes.Count - 2)) + 2;
            }
            PlanRouteToNode(nextGoal);

            // 赤ゾーン方向 (+X, Yaw=75〜105deg) を向く
            _targetYawDeg = 90.0f + UnityEngine.Random.Range(-15.0f, 15.0f);
        }

        private void PlanRouteToNode(int goalNode)
        {
            _navQueue.Clear();
            int startNode = FindNearestNodeIndex(transform.position);
            _currentNodeIndex = startNode;

            if (startNode == goalNode)
            {
                _navQueue.Enqueue(goalNode);
                return;
            }

            // BFS 最短経路探索
            int n = corridorNodes.Count;
            int[] parent = new int[n];
            bool[] visited = new bool[n];
            for (int i = 0; i < n; i++) parent[i] = -1;

            Queue<int> q = new Queue<int>();
            q.Enqueue(startNode);
            visited[startNode] = true;

            while (q.Count > 0)
            {
                int u = q.Dequeue();
                if (u == goalNode) break;

                foreach (int v in AdjacencyGraph[u])
                {
                    if (!visited[v])
                    {
                        visited[v] = true;
                        parent[v] = u;
                        q.Enqueue(v);
                    }
                }
            }

            List<int> path = new List<int>();
            int curr = goalNode;
            while (curr != -1 && curr != startNode)
            {
                path.Add(curr);
                curr = parent[curr];
            }
            path.Reverse();

            foreach (int nodeIdx in path)
            {
                _navQueue.Enqueue(nodeIdx);
            }
        }

        private int FindNearestNodeIndex(Vector3 pos)
        {
            int bestIdx = 0;
            float bestDistSq = float.MaxValue;
            for (int i = 0; i < corridorNodes.Count; i++)
            {
                Vector3 d = corridorNodes[i] - pos;
                d.y = 0f;
                if (d.sqrMagnitude < bestDistSq)
                {
                    bestDistSq = d.sqrMagnitude;
                    bestIdx = i;
                }
            }
            return bestIdx;
        }

        private void OnDrawGizmosSelected()
        {
            // 青ゾーン外周の可視化
            Gizmos.color = new Color(0.1f, 0.5f, 1.0f, 0.8f);
            Vector3 center = new Vector3((blueZoneMinX + blueZoneMaxX) * 0.5f, 0.05f, (blueZoneMinZ + blueZoneMaxZ) * 0.5f);
            Vector3 size = new Vector3(blueZoneMaxX - blueZoneMinX, 0.05f, blueZoneMaxZ - blueZoneMinZ);
            Gizmos.DrawWireCube(center, size);

            // 障害物AABBの可視化
            Gizmos.color = new Color(1.0f, 0.2f, 0.2f, 0.8f);
            for (int i = 0; i < BlueZoneObstacles.Length; i++)
            {
                var ob = BlueZoneObstacles[i];
                Vector3 obCenter = new Vector3((ob.minX + ob.maxX) * 0.5f, 0.25f, (ob.minZ + ob.maxZ) * 0.5f);
                Vector3 obSize = new Vector3(ob.maxX - ob.minX + robotBoundingRadius * 2f, 0.5f, ob.maxZ - ob.minZ + robotBoundingRadius * 2f);
                Gizmos.DrawWireCube(obCenter, obSize);
            }

            // 安全回廊グラフの可視化
            Gizmos.color = Color.cyan;
            if (corridorNodes != null)
            {
                for (int u = 0; u < corridorNodes.Count && u < AdjacencyGraph.Length; u++)
                {
                    Gizmos.DrawSphere(corridorNodes[u] + Vector3.up * 0.1f, 0.12f);
                    foreach (int v in AdjacencyGraph[u])
                    {
                        if (v < corridorNodes.Count && u < v)
                        {
                            Gizmos.DrawLine(corridorNodes[u] + Vector3.up * 0.1f, corridorNodes[v] + Vector3.up * 0.1f);
                        }
                    }
                }
            }

            // リロードゾーンの可視化
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(reloadPosition + Vector3.up * 0.1f, new Vector3(0.95f, 0.15f, 0.95f));
        }
    }
}
