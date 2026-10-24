using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace RobotSim.Opponent
{
    public enum OpponentState
    {
        Patrol,
        HeadingToReload,
        Reloading
    }

    [RequireComponent(typeof(Rigidbody))]
    public class OpponentRobotController : MonoBehaviour
    {
        [Header("Movement Settings")]
        [SerializeField] private float moveSpeed = 0.8f;            // 巡回速度 (m/s)
        [SerializeField] private float reloadApproachSpeed = 0.6f;  // リロード進入速度 (m/s)
        [SerializeField] private float rotateSpeedDeg = 90.0f;      // 旋回速度 (deg/s)
        [SerializeField] private float arrivalThreshold = 0.25f;    // ウェイポイント到達判定距離 (m)

        [Header("FSM Timing")]
        [SerializeField] private float patrolDuration = 20.0f;      // 巡回時間 (秒)
        [SerializeField] private float reloadWaitDuration = 4.0f;   // リロードゾーンでの補充待機時間 (秒)

        [Header("Reload Zone (Blue Desk Replenish Area)")]
        // 青ゾーンの補充スポット机 (desk_replenish_blue: X=-4.47, Z=1.08) 横のスタートゾーン付近
        [SerializeField] private Vector3 reloadPosition = new Vector3(-3.6f, 0.05f, 1.2f);
        [SerializeField] private float reloadHeadingYawDeg = 180f;  // 机に向かう向き (度)

        [Header("Patrol Waypoints (Blue Zone)")]
        [SerializeField] private List<Vector3> patrolWaypoints = new List<Vector3>
        {
            new Vector3(0.0f, 0.05f, 2.5f),    // 青フィールド中央
            new Vector3(2.5f, 0.05f, 3.8f),    // 青右奥 (机2付近)
            new Vector3(-2.0f, 0.05f, 3.8f),   // 青左奥 (机1付近)
            new Vector3(1.2f, 0.05f, 1.8f),    // 青手前中央 (固定バケツ1付近)
            new Vector3(2.8f, 0.05f, 2.2f),    // 青手前右 (固定バケツ3付近)
            new Vector3(-1.0f, 0.05f, 2.0f)    // 青手前左 (固定バケツ2付近)
        };

        [Header("ROS 2 Control (Optional Override)")]
        [SerializeField] private bool enableRosTeleop = false;
        [SerializeField] private string cmdVelTopic = "/opponent/cmd_vel";

        public OpponentState CurrentState { get; private set; } = OpponentState.Patrol;
        public float StateTimer { get; private set; } = 0f;
        public Vector3 CurrentVelocity { get; private set; } = Vector3.zero;

        private Rigidbody _rb;
        private int _currentWaypointIndex = 0;
        private float _stateElapsedTime = 0f;

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
            _stateElapsedTime += dt;

            // ROS 2 手動テレロップが有効かつ直近コマンド受信中の場合
            if (enableRosTeleop && (Time.timeAsDouble - _lastRosCmdTime < 0.5))
            {
                Vector3 moveDelta = transform.rotation * _rosCmdLinear * dt;
                _rb.MovePosition(ClampToBlueZone(transform.position + moveDelta));
                float yawDelta = _rosCmdAngularZ * Mathf.Rad2Deg * dt;
                _rb.MoveRotation(Quaternion.Euler(0f, transform.eulerAngles.y + yawDelta, 0f));
                CurrentVelocity = _rosCmdLinear;
                return;
            }

            // 自律 FSM
            switch (CurrentState)
            {
                case OpponentState.Patrol:
                    UpdatePatrol(dt);
                    break;

                case OpponentState.HeadingToReload:
                    UpdateHeadingToReload(dt);
                    break;

                case OpponentState.Reloading:
                    UpdateReloading(dt);
                    break;
            }
        }

        private void SetState(OpponentState newState)
        {
            CurrentState = newState;
            _stateElapsedTime = 0f;

            switch (newState)
            {
                case OpponentState.Patrol:
                    // 最も近い、または次のウェイポイントを選択
                    _currentWaypointIndex = (_currentWaypointIndex + 1) % Mathf.Max(1, patrolWaypoints.Count);
                    break;

                case OpponentState.HeadingToReload:
                    break;

                case OpponentState.Reloading:
                    CurrentVelocity = Vector3.zero;
                    break;
            }
        }

        private void UpdatePatrol(float dt)
        {
            if (patrolWaypoints == null || patrolWaypoints.Count == 0) return;

            Vector3 targetWp = patrolWaypoints[_currentWaypointIndex];
            Vector3 toTarget = targetWp - transform.position;
            toTarget.y = 0f;
            float dist = toTarget.magnitude;

            if (dist < arrivalThreshold)
            {
                // 次のウェイポイントへ
                _currentWaypointIndex = (_currentWaypointIndex + UnityEngine.Random.Range(1, 3)) % patrolWaypoints.Count;
            }

            // 移動
            MoveTowardsTarget(targetWp, moveSpeed, dt);

            // 巡回時間満了でリロードゾーンへ
            if (_stateElapsedTime >= patrolDuration)
            {
                SetState(OpponentState.HeadingToReload);
            }
        }

        private void UpdateHeadingToReload(float dt)
        {
            Vector3 toReload = reloadPosition - transform.position;
            toReload.y = 0f;
            float dist = toReload.magnitude;

            if (dist < arrivalThreshold)
            {
                SetState(OpponentState.Reloading);
                return;
            }

            MoveTowardsTarget(reloadPosition, reloadApproachSpeed, dt);
        }

        private void UpdateReloading(float dt)
        {
            CurrentVelocity = Vector3.zero;

            // 机に向くようにゆっくり旋回
            Quaternion targetRot = Quaternion.Euler(0f, reloadHeadingYawDeg, 0f);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, rotateSpeedDeg * dt);

            // 補充待機時間終了で巡回復帰
            if (_stateElapsedTime >= reloadWaitDuration)
            {
                SetState(OpponentState.Patrol);
            }
        }

        private void MoveTowardsTarget(Vector3 targetPos, float speed, float dt)
        {
            Vector3 dir = (targetPos - transform.position);
            dir.y = 0f;
            float dist = dir.magnitude;

            if (dist > 0.01f)
            {
                Vector3 moveDir = dir.normalized;
                Vector3 step = moveDir * (speed * dt);
                if (step.magnitude > dist) step = dir;

                Vector3 newPos = ClampToBlueZone(transform.position + step);
                _rb.MovePosition(newPos);
                CurrentVelocity = moveDir * speed;

                // 進行方向へスムーズに回転
                Quaternion targetRot = Quaternion.LookRotation(moveDir, Vector3.up);
                _rb.MoveRotation(Quaternion.RotateTowards(transform.rotation, targetRot, rotateSpeedDeg * dt));
            }
            else
            {
                CurrentVelocity = Vector3.zero;
            }
        }

        private Vector3 ClampToBlueZone(Vector3 pos)
        {
            // 青フィールド外周フェンスおよび中央教壇の制約 (X: [-4.2, +4.2], Z: [0.65, 5.0])
            pos.x = Mathf.Clamp(pos.x, -4.2f, 4.2f);
            pos.z = Mathf.Clamp(pos.z, 0.65f, 5.0f);
            pos.y = 0.05f; // 床面接地
            return pos;
        }

        private void OnDrawGizmosSelected()
        {
            // ウェイポイントの可視化
            Gizmos.color = Color.cyan;
            if (patrolWaypoints != null)
            {
                for (int i = 0; i < patrolWaypoints.Count; i++)
                {
                    Gizmos.DrawSphere(patrolWaypoints[i], 0.15f);
                    if (i < patrolWaypoints.Count - 1)
                    {
                        Gizmos.DrawLine(patrolWaypoints[i], patrolWaypoints[i + 1]);
                    }
                }
            }

            // リロードゾーンの可視化
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(reloadPosition, new Vector3(0.8f, 0.1f, 0.8f));
        }
    }
}
