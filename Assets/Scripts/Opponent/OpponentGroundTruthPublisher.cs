using System;
using UnityEngine;
using ROS2;
using RobotSim.Sensors;

namespace RobotSim.Opponent
{
    public class OpponentGroundTruthPublisher : MonoBehaviour
    {
        [Header("TF Settings")]
        [SerializeField] private string parentFrame = "map";
        [SerializeField] private string childFrame = "opponent_bucket_gt";
        [SerializeField] private float publishRateHz = 20.0f;
        // The ego robot owns base_link, so the opponent needs distinct frame names.
        [SerializeField] private string baseFrame = "opponent_base_link";
        [SerializeField] private string bucketOpeningFrame = "opponent_bucket_opening";
        [SerializeField] private string bucketBottomFrame = "opponent_bucket_bottom";

        private Transform _base;
        private Vector3 _openingLocal;
        private Vector3 _bottomLocal;
        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<tf2_msgs.msg.TFMessage> _pubTf;
        private IPublisher<geometry_msgs.msg.PoseStamped> _pubPose;

        private double _lastPublishTime;

        private void Start()
        {
            var controller = GetComponentInParent<OpponentRobotController>();
            var meshFilter = GetComponent<MeshFilter>();
            if (controller == null || meshFilter == null || meshFilter.sharedMesh == null)
            {
                Debug.LogError("[OpponentGroundTruthPublisher] Must be on a bucket mesh under OpponentRobotController.", this);
                enabled = false;
                return;
            }

            _base = controller.transform;
            Bounds bounds = meshFilter.sharedMesh.bounds;
            _openingLocal = bounds.center + Vector3.up * bounds.extents.y;
            _bottomLocal = bounds.center - Vector3.up * bounds.extents.y;
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

            double now = LivoxMid360Sim.CurrentSimTimeSec;
            double interval = 1.0 / Mathf.Max(1.0f, publishRateHz);
            if (now - _lastPublishTime < interval) return;
            _lastPublishTime = now;

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

            Quaternion rosRot = ToRos(uRot);

            LivoxMid360Sim.GetSimTime(out int sec, out uint nanosec);

            // 1. TF Message
            var tfMsg = new tf2_msgs.msg.TFMessage();
            var transformStamped = CreateTransform(parentFrame, childFrame, ToRos(uPos), rosRot, sec, nanosec);

            Vector3 basePos = _base.position;
            Quaternion baseRot = _base.rotation;
            Quaternion baseInverse = Quaternion.Inverse(baseRot);
            Quaternion bucketInBase = baseInverse * uRot;
            Vector3 opening = baseInverse * (transform.TransformPoint(_openingLocal) - basePos);
            Vector3 bottom = baseInverse * (transform.TransformPoint(_bottomLocal) - basePos);

            tfMsg.Transforms = new[]
            {
                transformStamped,
                CreateTransform(parentFrame, baseFrame, ToRos(basePos), ToRos(baseRot), sec, nanosec),
                CreateTransform(baseFrame, bucketOpeningFrame, ToRos(opening), ToRos(bucketInBase), sec, nanosec),
                CreateTransform(baseFrame, bucketBottomFrame, ToRos(bottom), ToRos(bucketInBase), sec, nanosec),
            };
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

        private static Vector3 ToRos(Vector3 unity) => new Vector3(unity.z, -unity.x, unity.y);

        private static Quaternion ToRos(Quaternion unity) => new Quaternion(-unity.z, unity.x, -unity.y, unity.w);

        private static geometry_msgs.msg.TransformStamped CreateTransform(
            string parent, string child, Vector3 rosPosition, Quaternion rosRotation, int sec, uint nanosec)
        {
            var message = new geometry_msgs.msg.TransformStamped();
            message.Header.Stamp.Sec = sec;
            message.Header.Stamp.Nanosec = nanosec;
            message.Header.Frame_id = parent;
            message.Child_frame_id = child;
            message.Transform.Translation.X = rosPosition.x;
            message.Transform.Translation.Y = rosPosition.y;
            message.Transform.Translation.Z = rosPosition.z;
            message.Transform.Rotation.X = rosRotation.x;
            message.Transform.Rotation.Y = rosRotation.y;
            message.Transform.Rotation.Z = rosRotation.z;
            message.Transform.Rotation.W = rosRotation.w;
            return message;
        }
    }
}
