using System;
using UnityEngine;
using ROS2;
using RobotSim.Shooter;

namespace RobotSim.Opponent
{
    public class OpponentBucketGoalDetector : MonoBehaviour
    {
        [Header("Bucket Dimensions")]
        [SerializeField] private float bucketRadius = 0.1365f; // PO-24A 内径 273mm/2
        [SerializeField] private float bucketDepth = 0.255f;   // 深さ 255mm

        [Header("Score Stats")]
        [SerializeField] private int scorePerGoal = 100;
        public int TotalGoals { get; private set; } = 0;
        public int TotalScore => TotalGoals * scorePerGoal;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<std_msgs.msg.Int32> _pubGoalCount;

        private double _lastGoalTime = -10.0;
        private const double CooldownSec = 1.0; // 連続多重カウント防止

        private void Start()
        {
            InitROS2();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                _node = _ros2Unity.CreateNode("opponent_goal_detector");
                _pubGoalCount = _node.CreatePublisher<std_msgs.msg.Int32>("/opponent/bucket_goals");
            }
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }

            // 幾何学的判定フォールバック (コライダーなしのメッシュ雑巾対策)
            CheckGeometricClothIntersection();
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.name.ToLower().Contains("cloth") || other.CompareTag("DustCloth") || other.name.Contains("SimulatedDustCloth"))
            {
                TriggerGoal("ColliderTrigger");
            }
        }

        private void CheckGeometricClothIntersection()
        {
            var flyingCloths = SimpleClothShooter.GetAllFlyingClothPositions();
            if (flyingCloths.Count == 0) return;

            Vector3 bucketTopCenter = transform.position;
            for (int i = 0; i < flyingCloths.Count; i++)
            {
                Vector3 clothPos = flyingCloths[i];
                // バケツ上面からの相対ベクトル
                Vector3 diff = clothPos - bucketTopCenter;
                float horizontalDist = Mathf.Sqrt(diff.x * diff.x + diff.z * diff.z);
                float verticalOffset = diff.y; // 0が上面、-bucketDepthが底面

                // 開口部半径以内かつ上面から底面の間に入ったかを判定
                if (horizontalDist <= bucketRadius && verticalOffset <= 0.05f && verticalOffset >= -bucketDepth)
                {
                    TriggerGoal("GeometricIntersection");
                    break;
                }
            }
        }

        private void TriggerGoal(string source)
        {
            double now = Time.timeAsDouble;
            if (now - _lastGoalTime < CooldownSec) return;

            _lastGoalTime = now;
            TotalGoals++;

            Debug.Log($"<color=green><b>[GOAL!!]</b></color> 雑巾が相手の移動バケツに入りました! (+{scorePerGoal}点 / 合計得点: {TotalScore}点, 検知: {source})");

            if (_pubGoalCount != null)
            {
                var msg = new std_msgs.msg.Int32 { Data = TotalGoals };
                _pubGoalCount.Publish(msg);
            }
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            // バケツ開口部円筒
            Gizmos.DrawWireSphere(transform.position, bucketRadius);
            Gizmos.DrawLine(transform.position, transform.position - Vector3.up * bucketDepth);
        }
    }
}

