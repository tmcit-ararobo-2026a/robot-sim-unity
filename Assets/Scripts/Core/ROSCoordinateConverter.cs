using UnityEngine;

namespace RobotSim.Core
{
    /// <summary>
    /// Utility for converting between ROS REP-103 (X-forward, Y-left, Z-up)
    /// and Unity (Z-forward, X-right, Y-up) coordinate systems.
    /// </summary>
    public static class ROSCoordinateConverter
    {
        public static Vector3 ROSToUnityPosition(float rx, float ry, float rz)
        {
            return new Vector3(-ry, rz, rx);
        }

        public static Vector3 UnityToROSPosition(Vector3 u)
        {
            return new Vector3(u.z, -u.x, u.y);
        }

        public static Vector3 ROSToUnityVelocity(float vx, float vy, float vz)
        {
            return new Vector3(-vy, vz, vx);
        }

        public static Vector3 UnityToROSVelocity(Vector3 u)
        {
            return new Vector3(u.z, -u.x, u.y);
        }

        public static Quaternion ROSToUnityRotation(Quaternion q)
        {
            return new Quaternion(-q.y, q.z, q.x, -q.w);
        }

        public static Quaternion UnityToROSRotation(Quaternion q)
        {
            return new Quaternion(q.z, -q.x, q.y, -q.w);
        }
    }
}
