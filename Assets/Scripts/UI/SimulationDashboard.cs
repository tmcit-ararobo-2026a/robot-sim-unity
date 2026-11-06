using System;
using UnityEngine;
using ROS2;
using RobotSim.Core;
using RobotSim.Shooter;

namespace RobotSim.UI
{
    public enum TargetId : byte
    {
        StartZone = 0,
        FixedBucket1 = 1,
        FixedBucket2 = 2,
        FixedBucket3 = 3,
        Desk1 = 4,
        Desk2 = 5,
        Flag = 6,
        MovingBucket = 7,
    }

    public enum NavigationCommand : byte
    {
        Sleep = 0,
        Stanby = 1,
        Move = 2,
    }

    public enum NavigationStatus : byte
    {
        Stanby = 0,
        Ready = 1,
        Moving = 2,
        Tracking = 3,
        Goal = 4,
        Fail = 5,
    }

    /// <summary>
    /// NHK2026 Robot Simulation Dashboard.
    /// Emulates the gn10-mainboard state machine, periodic command publishing (20Hz/50ms),
    /// and full target/command protocol including Moving Bucket tracking and ROS 2 status feedback.
    /// </summary>
    public class SimulationDashboard : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private SimpleClothShooter shooter;
        [SerializeField] private Transform robotTransform;

        [Header("Protocol Settings")]
        [Tooltip("Periodic command publication interval in seconds (50ms = 20Hz matches gn10-mainboard)")]
        [SerializeField] private float publishInterval = 0.05f;
        [Tooltip("Operation timeout in seconds (200ms matches gn10-mainboard)")]
        [SerializeField] private float operationTimeoutSec = 0.2f;

        // ROS 2 Communication
        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<std_msgs.msg.UInt8> _navCmdPub;
        private IPublisher<std_msgs.msg.UInt8> _targetIdPub;
        private IPublisher<std_msgs.msg.UInt8> _feedbackNavStatusPub;
        private ISubscription<std_msgs.msg.UInt8> _subOperationNavStatus;
        private ISubscription<std_msgs.msg.Bool> _subLocalizationSuccess;
        private ISubscription<std_msgs.msg.Float32> _subBeltLauncherSpeed;

        // State Machine (gn10-mainboard compatible)
        private bool _isAutoMode = true;
        private TargetId _targetId = TargetId.StartZone;
        private NavigationCommand _navCommand = NavigationCommand.Stanby;
        private NavigationStatus _currentStatus = NavigationStatus.Stanby;

        private readonly System.Diagnostics.Stopwatch _operationWatch = System.Diagnostics.Stopwatch.StartNew();
        private double _lastOperationReceivedSec = -10.0;
        private bool _operationTimeout = false;
        private bool _localizationSuccess = true;
        private bool _isMovingBucketHolding = false;

        private float _publishTimer = 0f;
        private float _receivedBeltLauncherSpeed = 0f;
        private float _manualShootSpeedRatio = 0.5f;

        public TargetId CurrentTargetId => _targetId;
        public NavigationCommand CurrentNavCommand => _navCommand;
        public NavigationStatus CurrentNavStatus => _currentStatus;
        public bool IsAutoMode => _isAutoMode;
        public float ReceivedBeltLauncherSpeed => _receivedBeltLauncherSpeed;

        private void Awake()
        {
            Application.runInBackground = true;
        }

        private void Start()
        {
            if (robotTransform == null) robotTransform = transform;
            if (shooter == null) shooter = FindFirstObjectByType<SimpleClothShooter>();
            InitROS2();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                _node = _ros2Unity.CreateNode("sim_dashboard_node");

                // Publishers compatible with udp_node / mainboard
                _navCmdPub = _node.CreatePublisher<std_msgs.msg.UInt8>("/robot/navigation_command");
                _targetIdPub = _node.CreatePublisher<std_msgs.msg.UInt8>("/robot/target_id");
                _feedbackNavStatusPub = _node.CreatePublisher<std_msgs.msg.UInt8>("/robot/feedback/navigation_status");

                // Subscriptions from ROS 2 navigation stack
                _subOperationNavStatus = _node.CreateSubscription<std_msgs.msg.UInt8>(
                    "/robot/operation/navigation_status", OnOperationNavStatus);
                _subLocalizationSuccess = _node.CreateSubscription<std_msgs.msg.Bool>(
                    "/robot/operation/localization_success", OnLocalizationSuccess);
                _subBeltLauncherSpeed = _node.CreateSubscription<std_msgs.msg.Float32>(
                    "/robot/operation/belt_launcher_speed", OnBeltLauncherSpeed);

                Debug.Log("[SimulationDashboard] ROS 2 node initialized with periodic 50ms command loop.");
            }
        }

        private void OnOperationNavStatus(std_msgs.msg.UInt8 msg)
        {
            _lastOperationReceivedSec = _operationWatch.Elapsed.TotalSeconds;
            _operationTimeout = false;

            if (_isAutoMode)
            {
                byte clampedData = msg.Data > (byte)NavigationStatus.Fail ? (byte)NavigationStatus.Fail : msg.Data;
                var newStatus = (NavigationStatus)clampedData;
                if (!_localizationSuccess)
                {
                    _currentStatus = NavigationStatus.Fail;
                }
                else
                {
                    _currentStatus = newStatus;
                }

                // gn10-mainboard parity: Goal or Fail automatically resets command to Stanby
                // to allow immediately accepting the next target trigger
                if (_currentStatus == NavigationStatus.Goal || _currentStatus == NavigationStatus.Fail)
                {
                    if (_navCommand == NavigationCommand.Move && _targetId != TargetId.MovingBucket)
                    {
                        _navCommand = NavigationCommand.Stanby;
                    }
                }
            }
        }

        private void OnLocalizationSuccess(std_msgs.msg.Bool msg)
        {
            _localizationSuccess = msg.Data;
            if (!_localizationSuccess && _isAutoMode)
            {
                _currentStatus = NavigationStatus.Fail;
            }
        }

        private void OnBeltLauncherSpeed(std_msgs.msg.Float32 msg)
        {
            _receivedBeltLauncherSpeed = msg.Data;
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }

            HandleKeyboardInput();
            UpdateStateMachine();
        }

        private void FixedUpdate()
        {
            // Periodic command publication (gn10-mainboard feedback loop parity: 20Hz / 50ms)
            // Using FixedUpdate ensures steady 50ms intervals regardless of editor window focus or render framerate.
            _publishTimer += Time.fixedDeltaTime;
            if (_publishTimer >= publishInterval)
            {
                _publishTimer = 0f;
                PublishPeriodicCommand();
            }
        }

        private void UpdateStateMachine()
        {
            double now = _operationWatch.Elapsed.TotalSeconds;

            if (!_isAutoMode)
            {
                // Manual mode: Sleep command and Stanby status
                _navCommand = NavigationCommand.Sleep;
                _currentStatus = NavigationStatus.Stanby;
                return;
            }

            // Watchdog: Communication timeout with Jetson/ROS2
            if (_lastOperationReceivedSec > 0.0 && (now - _lastOperationReceivedSec) > operationTimeoutSec)
            {
                _operationTimeout = true;
                _currentStatus = NavigationStatus.Fail;
                _navCommand = NavigationCommand.Sleep;
            }

            // MovingBucket hold state handling:
            // gn10-mainboard spec: Move while held, Stanby when released
            if (_isMovingBucketHolding)
            {
                _targetId = TargetId.MovingBucket;
                _navCommand = NavigationCommand.Move;
            }
            else if (_targetId == TargetId.MovingBucket && _navCommand == NavigationCommand.Move)
            {
                _navCommand = NavigationCommand.Stanby;
            }
        }

        private void HandleKeyboardInput()
        {
            // Toggle Auto/Manual Mode: 'M' key
            if (Input.GetKeyDown(KeyCode.M))
            {
                SetAutoMode(!_isAutoMode);
            }

            // Cancel / Standby: 'C' or Backspace
            if (Input.GetKeyDown(KeyCode.C) || Input.GetKeyDown(KeyCode.Backspace))
            {
                RequestStandby();
            }

            if (_isAutoMode)
            {
                // Numeric keys 0-6: Fixed targets trigger Move
                if (Input.GetKeyDown(KeyCode.Alpha0)) TriggerFixedTarget(TargetId.StartZone);
                if (Input.GetKeyDown(KeyCode.Alpha1)) TriggerFixedTarget(TargetId.FixedBucket1);
                if (Input.GetKeyDown(KeyCode.Alpha2)) TriggerFixedTarget(TargetId.FixedBucket2);
                if (Input.GetKeyDown(KeyCode.Alpha3)) TriggerFixedTarget(TargetId.FixedBucket3);
                if (Input.GetKeyDown(KeyCode.Alpha4)) TriggerFixedTarget(TargetId.Desk1);
                if (Input.GetKeyDown(KeyCode.Alpha5)) TriggerFixedTarget(TargetId.Desk2);
                if (Input.GetKeyDown(KeyCode.Alpha6)) TriggerFixedTarget(TargetId.Flag);

                // 'T' key or '7': MovingBucket tracking (Hold to Move, Release to Standby)
                if (Input.GetKeyDown(KeyCode.T) || Input.GetKeyDown(KeyCode.Alpha7))
                {
                    StartMovingBucketTracking();
                }
                if (Input.GetKeyUp(KeyCode.T) || Input.GetKeyUp(KeyCode.Alpha7))
                {
                    StopMovingBucketTracking();
                }
            }

            // Manual Shoot Cloth: Space key
            if (Input.GetKeyDown(KeyCode.Space))
            {
                TriggerShoot();
            }
        }

        public void SetAutoMode(bool auto)
        {
            bool prev = _isAutoMode;
            _isAutoMode = auto;

            if (prev && !_isAutoMode)
            {
                // Switched to manual: Sleep immediately
                _navCommand = NavigationCommand.Sleep;
                _currentStatus = NavigationStatus.Stanby;
                _isMovingBucketHolding = false;
            }
            else if (!prev && _isAutoMode)
            {
                // Switched to auto: Transition from Sleep to Stanby
                if (_navCommand == NavigationCommand.Sleep)
                {
                    _navCommand = NavigationCommand.Stanby;
                }
            }
            PublishPeriodicCommand();
            Debug.Log($"[SimulationDashboard] Mode changed to: {(_isAutoMode ? "AUTO" : "MANUAL")}");
        }

        public void TriggerFixedTarget(TargetId target)
        {
            if (!_isAutoMode) SetAutoMode(true);
            _isMovingBucketHolding = false;
            _targetId = target;
            _navCommand = NavigationCommand.Move;
            PublishPeriodicCommand();
            Debug.Log($"[SimulationDashboard] Triggered Fixed Target: {target} (cmd={_navCommand})");
        }

        public void StartMovingBucketTracking()
        {
            if (!_isAutoMode) SetAutoMode(true);
            _isMovingBucketHolding = true;
            _targetId = TargetId.MovingBucket;
            _navCommand = NavigationCommand.Move;
            PublishPeriodicCommand();
            Debug.Log("[SimulationDashboard] MovingBucket tracking STARTED (HOLD)");
        }

        public void StopMovingBucketTracking()
        {
            _isMovingBucketHolding = false;
            if (_targetId == TargetId.MovingBucket)
            {
                _navCommand = NavigationCommand.Stanby;
                PublishPeriodicCommand();
                Debug.Log("[SimulationDashboard] MovingBucket tracking STOPPED (Released to Stanby)");
            }
        }

        public void RequestStandby()
        {
            _isMovingBucketHolding = false;
            _navCommand = NavigationCommand.Stanby;
            PublishPeriodicCommand();
            Debug.Log("[SimulationDashboard] Requested Stanby (Cancel / Stop)");
        }

        public void RequestSleep()
        {
            _isMovingBucketHolding = false;
            _navCommand = NavigationCommand.Sleep;
            _currentStatus = NavigationStatus.Stanby;
            PublishPeriodicCommand();
            Debug.Log("[SimulationDashboard] Requested Sleep");
        }

        private void TriggerShoot()
        {
            if (shooter != null)
            {
                if (_receivedBeltLauncherSpeed > 0.1f)
                {
                    shooter.ShootClothWithBeltSpeed(_receivedBeltLauncherSpeed);
                }
                else
                {
                    shooter.ShootCloth(_manualShootSpeedRatio);
                }
            }
        }

        private void PublishPeriodicCommand()
        {
            if (_node == null) return;

            if (_navCmdPub != null)
            {
                var msg = new std_msgs.msg.UInt8 { Data = (byte)_navCommand };
                _navCmdPub.Publish(msg);
            }
            if (_targetIdPub != null)
            {
                var msg = new std_msgs.msg.UInt8 { Data = (byte)_targetId };
                _targetIdPub.Publish(msg);
            }
            if (_feedbackNavStatusPub != null)
            {
                var msg = new std_msgs.msg.UInt8 { Data = (byte)_currentStatus };
                _feedbackNavStatusPub.Publish(msg);
            }
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(10, 10, 360, 560), GUI.skin.box);
            GUILayout.Label("<b>NHK 2026 Robot Simulation Dashboard</b>");
            GUILayout.Space(4);

            // 1. Robot Pose
            if (robotTransform != null)
            {
                Vector3 rosPos = ROSCoordinateConverter.UnityToROSPosition(robotTransform.position);
                float yaw = -robotTransform.eulerAngles.y;
                GUILayout.Label($"Pose (ROS): X={rosPos.x:F2}m, Y={rosPos.y:F2}m, Z={rosPos.z:F2}m | {yaw:F1}°");
            }

            GUILayout.Space(6);
            // 2. Mode & Status Bar
            GUILayout.BeginHorizontal();
            GUI.color = _isAutoMode ? Color.green : Color.yellow;
            if (GUILayout.Button(_isAutoMode ? "MODE: AUTO (M)" : "MODE: MANUAL (M)", GUILayout.Height(26)))
            {
                SetAutoMode(!_isAutoMode);
            }
            GUI.color = Color.white;
            GUILayout.EndHorizontal();

            // Status display
            Color statusColor = _currentStatus switch
            {
                NavigationStatus.Ready => Color.cyan,
                NavigationStatus.Moving => Color.green,
                NavigationStatus.Tracking => Color.magenta,
                NavigationStatus.Goal => Color.yellow,
                NavigationStatus.Fail => Color.red,
                _ => Color.gray,
            };
            GUI.color = statusColor;
            GUILayout.Label($"<b>Status:</b> {_currentStatus} | <b>Command:</b> {_navCommand} | <b>Target:</b> {_targetId}");
            GUI.color = Color.white;

            if (_operationTimeout)
            {
                GUI.color = Color.red;
                GUILayout.Label("⚠ ROS 2 Operation Timeout (>200ms)!");
                GUI.color = Color.white;
            }

            GUILayout.Space(6);
            GUILayout.Label("<b>1. Fixed Target Navigation (gn10-navigation)</b>");

            // Row 1: Buckets 1-3
            GUILayout.BeginHorizontal();
            DrawTargetButton("Bucket 1 (1)", TargetId.FixedBucket1);
            DrawTargetButton("Bucket 2 (2)", TargetId.FixedBucket2);
            DrawTargetButton("Bucket 3 (3)", TargetId.FixedBucket3);
            GUILayout.EndHorizontal();

            // Row 2: Desks & Flag & StartZone
            GUILayout.BeginHorizontal();
            DrawTargetButton("Start (0)", TargetId.StartZone);
            DrawTargetButton("Desk 1 (4)", TargetId.Desk1);
            DrawTargetButton("Desk 2 (5)", TargetId.Desk2);
            DrawTargetButton("Flag (6)", TargetId.Flag);
            GUILayout.EndHorizontal();

            GUILayout.Space(4);
            // Row 3: Moving Bucket (HOLD to Track)
            GUI.color = (_isMovingBucketHolding || (_targetId == TargetId.MovingBucket && _navCommand == NavigationCommand.Move))
                ? Color.magenta
                : Color.white;

            // Handle Hold button for Moving Bucket
            Rect movingBucketRect = GUILayoutUtility.GetRect(new GUIContent("🎯 MOVING BUCKET (Hold T / Click & Hold)"), GUI.skin.button, GUILayout.Height(30));
            Event e = Event.current;
            if (movingBucketRect.Contains(e.mousePosition))
            {
                if (e.type == EventType.MouseDown && e.button == 0)
                {
                    StartMovingBucketTracking();
                }
                else if (e.type == EventType.MouseUp && e.button == 0)
                {
                    StopMovingBucketTracking();
                }
            }
            GUI.Button(movingBucketRect, "🎯 MOVING BUCKET (Hold T / Click & Hold)");
            GUI.color = Color.white;

            GUILayout.Space(4);
            // Standby / Cancel / Sleep commands
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("CANCEL / Stanby (C)", GUILayout.Height(24)))
            {
                RequestStandby();
            }
            if (GUILayout.Button("Sleep (0)", GUILayout.Height(24)))
            {
                RequestSleep();
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(8);
            GUILayout.Label("<b>2. Cloth Shooter Simulation (belt_launcher_speed)</b>");
            if (_receivedBeltLauncherSpeed > 0.05f)
            {
                GUI.color = Color.cyan;
                GUILayout.Label($"ROS 2 Speed: <b>{_receivedBeltLauncherSpeed:F2} m/s</b> (/robot/operation/belt_launcher_speed)");
                GUI.color = Color.white;
            }
            else
            {
                GUILayout.Label($"Manual Ratio: {_manualShootSpeedRatio:F2} (Level {Mathf.RoundToInt(_manualShootSpeedRatio * 10)})");
                _manualShootSpeedRatio = GUILayout.HorizontalSlider(_manualShootSpeedRatio, 0.1f, 1.0f);
            }

            if (GUILayout.Button("SHOOT CLOTH (Space)", GUILayout.Height(30)))
            {
                TriggerShoot();
            }

            GUILayout.Space(6);
            GUILayout.Label("<b>Shortcuts & Teleop</b>");
            GUILayout.Label("0-6: Fixed Targets | T (Hold): Moving Bucket\nC: Cancel | M: Auto/Manual | WASD: Manual Move");

            GUILayout.EndArea();
        }

        private void DrawTargetButton(string label, TargetId target)
        {
            bool isCurrent = (_targetId == target && _navCommand == NavigationCommand.Move);
            if (isCurrent) GUI.color = Color.green;
            if (GUILayout.Button(label, GUILayout.Height(24)))
            {
                TriggerFixedTarget(target);
            }
            if (isCurrent) GUI.color = Color.white;
        }
    }
}
