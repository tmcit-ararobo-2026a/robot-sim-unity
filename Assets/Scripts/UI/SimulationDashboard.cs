using UnityEngine;
using ROS2;
using RobotSim.Core;
using RobotSim.Shooter;

namespace RobotSim.UI
{
    public class SimulationDashboard : MonoBehaviour
    {
        [SerializeField] private SimpleClothShooter shooter;
        [SerializeField] private Transform robotTransform;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<std_msgs.msg.UInt8> _navCmdPub;
        private IPublisher<std_msgs.msg.UInt8> _targetIdPub;

        private byte _selectedTargetId = 0;
        private float _shootSpeedRatio = 0.5f;

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
                _navCmdPub = _node.CreatePublisher<std_msgs.msg.UInt8>("/robot/navigation_command");
                _targetIdPub = _node.CreatePublisher<std_msgs.msg.UInt8>("/robot/target_id");
            }
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(10, 10, 320, 420), GUI.skin.box);
            GUILayout.Label("<b>NHK 2026 Robot Simulation Dashboard</b>");
            GUILayout.Space(5);

            // 1. Robot Pose
            if (robotTransform != null)
            {
                Vector3 rosPos = ROSCoordinateConverter.UnityToROSPosition(robotTransform.position);
                float yaw = -robotTransform.eulerAngles.y;
                GUILayout.Label($"Robot Position (ROS): X={rosPos.x:F2}m, Y={rosPos.y:F2}m, Z={rosPos.z:F2}m");
                GUILayout.Label($"Robot Heading: {yaw:F1}°");
            }

            GUILayout.Space(10);
            GUILayout.Label("<b>1. Autonomous Navigation (gn10-navigation)</b>");
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Target ID: {_selectedTargetId}", GUILayout.Width(90));
            for (byte i = 0; i <= 3; i++)
            {
                if (GUILayout.Button($"{i}", GUILayout.Width(35))) _selectedTargetId = i;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Standby (1)"))
            {
                SendNavCommand(1, _selectedTargetId);
            }
            if (GUILayout.Button("START MOVE (2)"))
            {
                SendNavCommand(2, _selectedTargetId);
            }
            if (GUILayout.Button("Sleep (0)"))
            {
                SendNavCommand(0, _selectedTargetId);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10);
            GUILayout.Label("<b>2. Cloth Shooter Simulation</b>");
            GUILayout.Label($"Belt Speed Ratio: {_shootSpeedRatio:F2} (Level {Mathf.RoundToInt(_shootSpeedRatio * 10)})");
            _shootSpeedRatio = GUILayout.HorizontalSlider(_shootSpeedRatio, 0.1f, 1.0f);

            if (GUILayout.Button("SHOOT CLOTH (Space)", GUILayout.Height(30)))
            {
                if (shooter != null) shooter.ShootCloth(_shootSpeedRatio);
            }

            GUILayout.Space(10);
            GUILayout.Label("<b>Manual Controls</b>");
            GUILayout.Label("WASD: Move Robot | Q/E: Rotate");

            GUILayout.EndArea();
        }

        private void SendNavCommand(byte cmd, byte targetId)
        {
            if (_navCmdPub != null)
            {
                std_msgs.msg.UInt8 cmdMsg = new std_msgs.msg.UInt8 { Data = cmd };
                _navCmdPub.Publish(cmdMsg);
            }
            if (_targetIdPub != null)
            {
                std_msgs.msg.UInt8 idMsg = new std_msgs.msg.UInt8 { Data = targetId };
                _targetIdPub.Publish(idMsg);
            }
            Debug.Log($"[SimulationDashboard] Sent navigation_command={cmd}, target_id={targetId}");
        }
    }
}
