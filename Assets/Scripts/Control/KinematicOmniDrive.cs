using System;
using UnityEngine;
using ROS2;

namespace RobotSim.Control
{
    [RequireComponent(typeof(Rigidbody))]
    public class KinematicOmniDrive : MonoBehaviour
    {
        [Header("Drive Limits")]
        [SerializeField] private float maxLinearSpeed = 2.0f;
        [SerializeField] private float maxAngularSpeed = 3.14f;

        public static Vector3 SharedRobotPosition { get; private set; }
        public static Quaternion SharedRobotRotation { get; private set; }
        public static float SharedRobotYawRad { get; private set; }
        public static Vector3 SharedLinearVelocityWorld { get; private set; }
        public static Vector3 SharedLinearAccelBase { get; private set; }
        public static float SharedAngularVelocityRadZ { get; private set; }

        private static readonly object StateLock = new object();
        private static readonly System.Diagnostics.Stopwatch CmdWatch = System.Diagnostics.Stopwatch.StartNew();
        private static Vector3 _targetLocalVelocity = Vector3.zero;
        private static float _targetAngularZ = 0.0f;
        private static Vector3 _currentLocalVelocity = Vector3.zero;
        private static float _currentAngularZ = 0.0f;
        private static double _lastRosCmdSec = -10.0;
        private static bool _manualKeyActive = false;
        private static bool _initialized = false;

        private Rigidbody _rb;
        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private ISubscription<geometry_msgs.msg.Twist> _subCmdVel;
        private ISubscription<geometry_msgs.msg.Twist> _subRobotOpCmdVel;

        private void Awake()
        {
            Application.runInBackground = true;
            _rb = GetComponent<Rigidbody>();
            _rb.useGravity = false;
            _rb.isKinematic = true;
            _rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

            lock (StateLock)
            {
                SharedRobotPosition = transform.position;
                SharedRobotRotation = transform.rotation;
                SharedRobotYawRad = -transform.eulerAngles.y * Mathf.Deg2Rad;
                SharedLinearVelocityWorld = Vector3.zero;
                SharedLinearAccelBase = Vector3.zero;
                SharedAngularVelocityRadZ = 0.0f;
                _currentLocalVelocity = Vector3.zero;
                _currentAngularZ = 0.0f;
                _targetLocalVelocity = Vector3.zero;
                _targetAngularZ = 0.0f;
                _initialized = true;
            }
        }

        private void Start()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                InitROS2();
            }
        }

        private void InitROS2()
        {
            _node = _ros2Unity.CreateNode("unity_omni_drive");
            _subCmdVel = _node.CreateSubscription<geometry_msgs.msg.Twist>(
                "/cmd_vel",
                OnTwistReceived
            );
            _subRobotOpCmdVel = _node.CreateSubscription<geometry_msgs.msg.Twist>(
                "/robot/operation/cmd_vel",
                OnTwistReceived
            );
            Debug.Log("[KinematicOmniDrive] Subscribed to /cmd_vel and /robot/operation/cmd_vel");
        }

        private void OnTwistReceived(geometry_msgs.msg.Twist twist)
        {
            float rx = (float)twist.Linear.X;
            float ry = (float)twist.Linear.Y;
            float rz = (float)twist.Angular.Z;

            Vector3 desiredLocal = new Vector3(-ry, 0f, rx);
            if (desiredLocal.magnitude > maxLinearSpeed)
            {
                desiredLocal = desiredLocal.normalized * maxLinearSpeed;
            }

            lock (StateLock)
            {
                _targetLocalVelocity = desiredLocal;
                _targetAngularZ = Mathf.Clamp(rz, -maxAngularSpeed, maxAngularSpeed);
                _lastRosCmdSec = CmdWatch.Elapsed.TotalSeconds;
            }
        }

        public static void StepKinematics(float dt)
        {
            if (!_initialized || dt <= 0f) return;

            lock (StateLock)
            {
                if (!_manualKeyActive && (CmdWatch.Elapsed.TotalSeconds - _lastRosCmdSec) > 0.25)
                {
                    _targetLocalVelocity = Vector3.zero;
                    _targetAngularZ = 0.0f;
                }

                Vector3 prevLocalVel = _currentLocalVelocity;
                float linAccelRate = _targetLocalVelocity.sqrMagnitude < 1e-4f ? 25.0f : 10.0f;
                float angAccelRate = Mathf.Abs(_targetAngularZ) < 1e-4f ? 25.0f : 10.0f;
                _currentLocalVelocity = Vector3.MoveTowards(_currentLocalVelocity, _targetLocalVelocity, linAccelRate * dt);
                _currentAngularZ = Mathf.MoveTowards(_currentAngularZ, _targetAngularZ, angAccelRate * dt);

                Vector3 localAccel = (_currentLocalVelocity - prevLocalVel) / dt;
                SharedLinearAccelBase = localAccel;

                SharedRobotYawRad += _currentAngularZ * dt;
                if (SharedRobotYawRad > Mathf.PI) SharedRobotYawRad -= 2.0f * Mathf.PI;
                else if (SharedRobotYawRad < -Mathf.PI) SharedRobotYawRad += 2.0f * Mathf.PI;

                float halfUnityYaw = -SharedRobotYawRad * 0.5f;
                SharedRobotRotation = new Quaternion(0f, Mathf.Sin(halfUnityYaw), 0f, Mathf.Cos(halfUnityYaw));

                Vector3 worldVel = SharedRobotRotation * _currentLocalVelocity;
                worldVel.y = 0f;
                SharedRobotPosition += worldVel * dt;
                SharedLinearVelocityWorld = worldVel;
                SharedAngularVelocityRadZ = _currentAngularZ;
            }
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }

            bool forward = Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow);
            bool backward = Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow);
            bool left = Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow);
            bool right = Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow);
            bool turnLeft = Input.GetKey(KeyCode.Q);
            bool turnRight = Input.GetKey(KeyCode.E);

            if (forward || backward || left || right || turnLeft || turnRight)
            {
                float keyZ = (forward ? 1f : 0f) - (backward ? 1f : 0f);
                float keyX = (right ? 1f : 0f) - (left ? 1f : 0f);
                float keyTurn = (turnRight ? -1f : 0f) + (turnLeft ? 1f : 0f);
                lock (StateLock)
                {
                    _manualKeyActive = true;
                    _targetLocalVelocity = new Vector3(keyX, 0f, keyZ).normalized * maxLinearSpeed;
                    _targetAngularZ = keyTurn * maxAngularSpeed;
                }
            }
            else if (_manualKeyActive)
            {
                lock (StateLock)
                {
                    _manualKeyActive = false;
                    _targetLocalVelocity = Vector3.zero;
                    _targetAngularZ = 0.0f;
                    _currentLocalVelocity = Vector3.zero;
                    _currentAngularZ = 0.0f;
                }
            }

            lock (StateLock)
            {
                transform.SetPositionAndRotation(SharedRobotPosition, SharedRobotRotation);
            }
        }
    }
}
