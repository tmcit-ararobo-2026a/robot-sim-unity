using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace RobotSim.Control
{
    // Keep the script identity so existing scenes retain their drive component.
    // Motion is simulated by a dynamic Rigidbody, not integrated into Transform.
    [RequireComponent(typeof(Rigidbody))]
    public class KinematicOmniDrive : MonoBehaviour
    {
        [Header("Drive Limits")]
        [SerializeField, Min(0f)] private float maxLinearSpeed = 2.0f;
        [SerializeField, Min(0f)] private float maxAngularSpeed = 3.14f;
        [Tooltip("Velocity command ramp in m/s²; full forward speed takes about 0.5 s.")]
        [SerializeField, Min(0.01f)] private float linearAcceleration = 4f;
        [SerializeField, Min(0.01f)] private float brakingAcceleration = 6f;
        [Tooltip("Angular command ramp in rad/s².")]
        [SerializeField, Min(0.01f)] private float angularAcceleration = 8f;
        [SerializeField, Min(0.01f)] private float angularBrakingAcceleration = 12f;
        [SerializeField, Min(0.01f)] private float commandTimeout = 0.25f;

        [Header("Motor Force Limits")]
        [Tooltip("Motor force / mass limit in m/s², separate from the comfortable command ramp.")]
        [SerializeField, Min(0.01f)] private float maxDriveAcceleration = 10f;
        [SerializeField, Min(0.01f)] private float maxYawDriveAcceleration = 16f;

        [Header("Chassis Contact")]
        [Tooltip("Low rolling resistance for the chassis-level omni-wheel approximation.")]
        [SerializeField, Range(0f, 1f)] private float rollingFriction = 0.05f;
        [SerializeField, HideInInspector] private bool bodyPhysicsBaked;

        public readonly struct MotionState
        {
            public readonly Vector3 Position;
            public readonly Quaternion Rotation;
            public readonly Vector3 LinearVelocityWorld;
            // Actual world acceleration expressed in the base frame, including free fall.
            public readonly Vector3 LinearAccelerationBase;
            public readonly float AngularVelocityRadZ;

            public MotionState(Vector3 position, Quaternion rotation, Vector3 velocity,
                Vector3 accelerationBase, float angularVelocityRadZ)
            {
                Position = position;
                Rotation = rotation;
                LinearVelocityWorld = velocity;
                LinearAccelerationBase = accelerationBase;
                AngularVelocityRadZ = angularVelocityRadZ;
            }
        }

        private static readonly object StateLock = new object();
        private static MotionState _state = new MotionState(Vector3.zero, Quaternion.identity,
            Vector3.zero, Vector3.zero, 0f);

        // ROS callbacks run off the main thread: copy one complete physical snapshot.
        public static MotionState GetMotionState()
        {
            lock (StateLock) return _state;
        }

        public static Vector3 SharedRobotPosition => GetMotionState().Position;
        public static Quaternion SharedRobotRotation => GetMotionState().Rotation;
        public static float SharedRobotYawRad => -Mathf.DeltaAngle(0f, GetMotionState().Rotation.eulerAngles.y) * Mathf.Deg2Rad;
        public static Vector3 SharedLinearVelocityWorld => GetMotionState().LinearVelocityWorld;
        public static Vector3 SharedLinearAccelBase => GetMotionState().LinearAccelerationBase;
        public static float SharedAngularVelocityRadZ => GetMotionState().AngularVelocityRadZ;

        private readonly object _commandLock = new object();
        private readonly System.Diagnostics.Stopwatch _commandWatch = System.Diagnostics.Stopwatch.StartNew();
        private Vector3 _targetLocalVelocity;
        private float _targetAngularZ;
        private double _lastRosCmdSec = -10;
        private bool _manualKeyActive;
        private bool _manualTranslationActive;
        private Quaternion _manualMovementRotation;
        private Vector3 _manualWorldVelocity;
        private float _manualAngularZ;
        private Vector3 _rampedLocalVelocity;
        private Vector3 _rampedWorldVelocity;
        private float _rampedAngularZ;
        private readonly HashSet<Collider> _groundContacts = new HashSet<Collider>();
        public bool IsGrounded => _groundContacts.Count > 0;

        private Rigidbody _rb;
        private PhysicsMaterial _rollingMaterial;
        private readonly List<Collider> _rollingColliders = new List<Collider>();
        private Vector3 _previousVelocityWorld;
        private Vector3 _accelerationWorld;
        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private ISubscription<geometry_msgs.msg.Twist> _subCmdVel;
        private ISubscription<geometry_msgs.msg.Twist> _subRobotOpCmdVel;

        private void Awake()
        {
            Application.runInBackground = true;
            _rb = GetComponent<Rigidbody>();
            if (!bodyPhysicsBaked) ConfigureBodyPhysics();
            AlignYawCenter();
            AlignChassisInertia();
            _rb.maxAngularVelocity = Mathf.Max(_rb.maxAngularVelocity, maxAngularSpeed);

            // A default box's static/patch friction locks strafing and yaw. Approximate
            // freely rolling omni wheels while keeping collision support and impulses.
            foreach (var collider in GetComponentsInChildren<Collider>())
            {
                if (collider.attachedRigidbody != _rb || collider.isTrigger || collider.sharedMaterial != null) continue;
                if (_rollingMaterial == null)
                {
                    _rollingMaterial = new PhysicsMaterial("Omni drive rolling contact")
                    {
                        staticFriction = rollingFriction,
                        dynamicFriction = rollingFriction,
                        frictionCombine = PhysicsMaterialCombine.Minimum,
                        bounciness = 0f,
                        bounceCombine = PhysicsMaterialCombine.Minimum
                    };
                }
                collider.sharedMaterial = _rollingMaterial;
                _rollingColliders.Add(collider);
            }
        }

        public void ConfigureBodyPhysics()
        {
            _rb = GetComponent<Rigidbody>();
            _rb.isKinematic = false;
            _rb.useGravity = true;
            _rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
            _rb.interpolation = RigidbodyInterpolation.Interpolate;
            _rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            _rb.maxAngularVelocity = Mathf.Max(_rb.maxAngularVelocity, maxAngularSpeed);
            AlignYawCenter();
            AlignChassisInertia();
            bodyPhysicsBaked = !Application.isPlaying;
        }

        private void AlignChassisInertia()
        {
            // PhysX angular locks use the inertia frame. The asymmetric CAD
            // compound tilts its principal axes, allowing slow roll/pitch drift
            // even with X/Z frozen. Preserve the moments about each chassis axis
            // and omit cross terms for this roll/pitch-locked approximation.
            // Unlock temporarily: the getter reports zero (infinite inertia) for
            // frozen axes, which would otherwise erase the yaw moment on boxes
            // whose automatically chosen principal axes are permuted.
            var constraints = _rb.constraints;
            _rb.constraints = RigidbodyConstraints.None;
            _rb.ResetInertiaTensor();
            Vector3 moments = _rb.inertiaTensor;
            Quaternion toPrincipal = Quaternion.Inverse(_rb.inertiaTensorRotation);
            Vector3 x = toPrincipal * Vector3.right;
            Vector3 y = toPrincipal * Vector3.up;
            Vector3 z = toPrincipal * Vector3.forward;
            _rb.inertiaTensorRotation = Quaternion.identity;
            _rb.inertiaTensor = new Vector3(
                Vector3.Dot(x, Vector3.Scale(moments, x)),
                Vector3.Dot(y, Vector3.Scale(moments, y)),
                Vector3.Dot(z, Vector3.Scale(moments, z)));
            _rb.constraints = constraints;
        }

        private void AlignYawCenter()
        {
            // Imported asymmetric CAD colliders put the automatic COM off the
            // base_link axis. Rigidbody velocity belongs to that COM, so applying
            // yaw otherwise makes the commanded base_link origin orbit around it.
            // Roll/pitch are locked: retain the computed height and align only the
            // horizontal mass center with the robot's commanded rotation axis.
            Vector3 center = _rb.centerOfMass;
            center.x = 0f;
            center.z = 0f;
            _rb.centerOfMass = center;
        }

        private void OnDestroy()
        {
            foreach (var collider in _rollingColliders)
                if (collider != null && collider.sharedMaterial == _rollingMaterial) collider.sharedMaterial = null;
            if (_rollingMaterial != null) Destroy(_rollingMaterial);
        }

        private void OnEnable()
        {
            _groundContacts.Clear();
            _manualKeyActive = false;
            _manualTranslationActive = false;
            _manualMovementRotation = _rb.rotation;
            _manualWorldVelocity = Vector3.zero;
            _manualAngularZ = 0f;
            _previousVelocityWorld = _rb.linearVelocity;
            _accelerationWorld = Vector3.zero;
            ResetCommandRampToBody();
            PublishMotionState();
        }

        private void Start()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            InitROS2();
        }

        private void InitROS2()
        {
            if (_ros2Unity == null || !_ros2Unity.Ok() || _node != null) return;
            _node = _ros2Unity.CreateNode("unity_omni_drive");
            _subCmdVel = _node.CreateSubscription<geometry_msgs.msg.Twist>("/cmd_vel", OnTwistReceived);
            _subRobotOpCmdVel = _node.CreateSubscription<geometry_msgs.msg.Twist>(
                "/robot/operation/cmd_vel", OnTwistReceived);
            Debug.Log("[KinematicOmniDrive] Dynamic Rigidbody drive subscribed to /cmd_vel and /robot/operation/cmd_vel");
        }

        private void OnTwistReceived(geometry_msgs.msg.Twist twist)
        {
            SetVelocityCommand(new Vector3((float)twist.Linear.X, (float)twist.Linear.Y, 0f),
                (float)twist.Angular.Z);
        }

        // ROS coordinates: x forward, y left, positive angular z counterclockwise.
        public void SetVelocityCommand(Vector3 linearRos, float angularRosZ)
        {
            if (!IsFinite(linearRos.x) || !IsFinite(linearRos.y) || !IsFinite(angularRosZ)) return;
            Vector3 desiredLocal = Vector3.ClampMagnitude(new Vector3(-linearRos.y, 0f, linearRos.x), maxLinearSpeed);
            lock (_commandLock)
            {
                _targetLocalVelocity = desiredLocal;
                _targetAngularZ = Mathf.Clamp(angularRosZ, -maxAngularSpeed, maxAngularSpeed);
                _lastRosCmdSec = _commandWatch.Elapsed.TotalSeconds;
            }
        }

        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

        private void Update()
        {
            InitROS2();
            bool forward = Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow);
            bool backward = Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow);
            bool left = Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow);
            bool right = Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow);
            bool turnLeft = Input.GetKey(KeyCode.Q);
            bool turnRight = Input.GetKey(KeyCode.E);
            bool active = forward || backward || left || right || turnLeft || turnRight;
            Vector3 direction = new Vector3((right ? 1f : 0f) - (left ? 1f : 0f), 0f,
                (forward ? 1f : 0f) - (backward ? 1f : 0f));
            UpdateManualCommand(direction, (turnLeft ? 1f : 0f) - (turnRight ? 1f : 0f), active);
        }

        private void UpdateManualCommand(Vector3 direction, float turn, bool active)
        {
            if (_manualKeyActive && !active)
            {
                // Releasing manual control brakes; do not revive an earlier ROS command.
                lock (_commandLock) _lastRosCmdSec = -10;
            }
            _manualKeyActive = active;
            bool translating = direction.sqrMagnitude > 1e-4f;
            // Keep the keyboard's movement axes fixed for this continuous move.
            // Turning changes the body heading without bending the chosen path.
            if (translating && !_manualTranslationActive) _manualMovementRotation = _rb.rotation;
            _manualTranslationActive = translating;
            _manualWorldVelocity = _manualMovementRotation * direction.normalized * maxLinearSpeed;
            _manualAngularZ = turn * maxAngularSpeed;
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            Vector3 velocity = _rb.linearVelocity;
            _accelerationWorld = (velocity - _previousVelocityWorld) / dt;
            _previousVelocityWorld = velocity;
            PublishMotionState();

            Vector3 localVelocity;
            float angularZ;
            lock (_commandLock)
            {
                bool fresh = _commandWatch.Elapsed.TotalSeconds - _lastRosCmdSec <= commandTimeout;
                localVelocity = fresh ? _targetLocalVelocity : Vector3.zero;
                angularZ = fresh ? _targetAngularZ : 0f;
            }
            if (_manualKeyActive)
            {
                angularZ = _manualAngularZ;
            }

            // Ground contact supplies traction. No drive forces while falling or airborne.
            if (!IsGrounded)
            {
                ResetCommandRampToBody();
                return;
            }
            bool stopping = _manualKeyActive
                ? _manualWorldVelocity.sqrMagnitude < 1e-4f : localVelocity.sqrMagnitude < 1e-4f;
            float linearRate = stopping ? brakingAcceleration : linearAcceleration;
            bool slowingYaw = Mathf.Abs(angularZ) < Mathf.Abs(_rampedAngularZ) || angularZ * _rampedAngularZ < 0f;
            float angularRate = slowingYaw || Mathf.Abs(angularZ) < 1e-4f
                ? angularBrakingAcceleration : angularAcceleration;
            if (_manualKeyActive || stopping)
            {
                // Ramp and brake keyboard motion in the world frame, including
                // the coast after key release. Keep ROS ramps in the body frame.
                Vector3 targetWorld = _manualKeyActive ? _manualWorldVelocity : Vector3.zero;
                _rampedWorldVelocity = Vector3.MoveTowards(_rampedWorldVelocity, targetWorld, linearRate * dt);
                _rampedLocalVelocity = Quaternion.Inverse(_rb.rotation) * _rampedWorldVelocity;
            }
            else
            {
                _rampedLocalVelocity = Vector3.MoveTowards(_rampedLocalVelocity, localVelocity, linearRate * dt);
                _rampedWorldVelocity = _rb.rotation * _rampedLocalVelocity;
            }
            _rampedAngularZ = Mathf.MoveTowards(_rampedAngularZ, angularZ, angularRate * dt);

            Vector3 desiredWorld = _rampedWorldVelocity;
            Vector3 planarVelocity = new Vector3(velocity.x, 0f, velocity.z);
            Vector3 driveAcceleration = Vector3.ClampMagnitude((desiredWorld - planarVelocity) / dt, maxDriveAcceleration);
            _rb.AddForce(driveAcceleration * _rb.mass, ForceMode.Force);

            float desiredYawVelocity = -_rampedAngularZ;
            float yawAcceleration = Mathf.Clamp((desiredYawVelocity - _rb.angularVelocity.y) / dt,
                -maxYawDriveAcceleration, maxYawDriveAcceleration);
            _rb.AddTorque(Vector3.up * yawAcceleration, ForceMode.Acceleration);
        }

        private void ResetCommandRampToBody()
        {
            _rampedLocalVelocity = Quaternion.Inverse(_rb.rotation) * _rb.linearVelocity;
            _rampedLocalVelocity.y = 0f;
            _rampedWorldVelocity = _rb.rotation * _rampedLocalVelocity;
            _rampedAngularZ = -_rb.angularVelocity.y;
        }

        private void LateUpdate()
        {
            // Include the completed physics step. Interpolated Transform is only for rendering.
            PublishMotionState();
        }

        private void PublishMotionState()
        {
            Quaternion rotation = _rb.rotation;
            var state = new MotionState(_rb.position, rotation, _rb.linearVelocity,
                Quaternion.Inverse(rotation) * _accelerationWorld, -_rb.angularVelocity.y);
            lock (StateLock) _state = state;
        }

        private void OnCollisionEnter(Collision collision) => UpdateGroundContact(collision);
        private void OnCollisionStay(Collision collision) => UpdateGroundContact(collision);
        private void OnCollisionExit(Collision collision) => _groundContacts.Remove(collision.collider);

        private void UpdateGroundContact(Collision collision)
        {
            for (int i = 0; i < collision.contactCount; i++)
            {
                if (collision.GetContact(i).normal.y > 0.5f)
                {
                    _groundContacts.Add(collision.collider);
                    return;
                }
            }
            _groundContacts.Remove(collision.collider);
        }
    }
}
