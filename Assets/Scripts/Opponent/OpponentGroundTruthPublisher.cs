using System;
using UnityEngine;
using ROS2;

namespace RobotSim.Opponent
{
    public class OpponentGroundTruthPublisher : MonoBehaviour
    {
        [Header("TF Settings")]
        [SerializeField] private string parentFrame = "map";
        [SerializeField] private string childFrame = "opponent_bucket_gt";
        [SerializeField] private float publishRateHz = 20.0f;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<tf2_msgs.msg.TFMessage> _pubTf;
        private IPublisher<geometry_msgs.msg.PoseStamped> _pubPose;

        private float _lastPublishTime = 0f;

        private void Start()
        {
            InitROS2();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok())
            {
                _node = _ros2Unity.CreateNode("opponent_gt_publisher");
                _pubTf = _node.CreatePublisher<tf2_msgs.msg.TFMessage>("/tf");
                _pubPose = _node.CreatePublisher<geometry_msgs.msg.PoseStamped>("/opponent/bucket_pose_gt");
            }
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }

            if (_node == null || _pubTf == null) return;

            float interval = 1.0f / Mathf.Max(1.0f, publishRateHz);
            if (Time.time - _lastPublishTime < interval) return;
            _lastPublishTime = Time.time;

            PublishGroundTruth();
        }

        private void PublishGroundTruth()
        {
            Vector3 uPos = transform.position;
            Quaternion uRot = transform.rotation;

            // Unity -> ROS 2 座標系変換: X_ros = Z_u, Y_ros = -X_u, Z_ros = Y_u
            double rosX = uPos.z;
            double rosY = -uPos.x;
            double rosZ = uPos.y;

            Quaternion rosRot = new Quaternion(-uRot.z, uRot.x, -uRot.y, uRot.w);

            int sec = (int)Time.time;
            uint nanosec = (uint)((Time.time - sec) * 1e9);

            // 1. TF Message
            var tfMsg = new tf2_msgs.msg.TFMessage();
            var transformStamped = new geometry_msgs.msg.TransformStamped();
            transformStamped.Header.Stamp.Sec = sec;
            transformStamped.Header.Stamp.Nanosec = nanosec;
            transformStamped.Header.Frame_id = parentFrame;
            transformStamped.Child_frame_id = childFrame;

            transformStamped.Transform.Translation.X = rosX;
            transformStamped.Transform.Translation.Y = rosY;
            transformStamped.Transform.Translation.Z = rosZ;
            transformStamped.Transform.Rotation.X = rosRot.x;
            transformStamped.Transform.Rotation.Y = rosRot.y;
            transformStamped.Transform.Rotation.Z = rosRot.z;
            transformStamped.Transform.Rotation.W = rosRot.w;

            tfMsg.Transforms = new[] { transformStamped };
            _pubTf.Publish(tfMsg);

            // 2. PoseStamped
            if (_pubPose != null)
            {
                var poseMsg = new geometry_msgs.msg.PoseStamped();
                poseMsg.Header.Stamp.Sec = sec;
                poseMsg.Header.Stamp.Nanosec = nanosec;
                poseMsg.Header.Frame_id = parentFrame;
                poseMsg.Pose.Position.X = rosX;
                poseMsg.Pose.Position.Y = rosY;
                poseMsg.Pose.Position.Z = rosZ;
                poseMsg.Pose.Orientation.X = rosRot.x;
                poseMsg.Pose.Orientation.Y = rosRot.y;
                poseMsg.Pose.Orientation.Z = rosRot.z;
                poseMsg.Pose.Orientation.W = rosRot.w;
                _pubPose.Publish(poseMsg);
            }
        }
    }
}
