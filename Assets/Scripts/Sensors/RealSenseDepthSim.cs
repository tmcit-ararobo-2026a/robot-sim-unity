using System;
using System.Diagnostics;
using Unity.Collections;
using UnityEngine;
using ROS2;
using RobotSim.Control;
using RobotSim.Shooter;
using Debug = UnityEngine.Debug;

namespace RobotSim.Sensors
{
    [RequireComponent(typeof(Camera))]
    public class RealSenseDepthSim : MonoBehaviour
    {
        [Header("Camera Specs")]
        [SerializeField] private int width = 640;
        [SerializeField] private int height = 480;
        [SerializeField] private float frameRate = 30.0f;
        [SerializeField] private string frameId = "camera_depth_optical_frame";

        private Camera _cam;
        private RenderTexture _rtColor;
        private RenderTexture _rtDepth;
        private Texture2D _texColor;
        private Texture2D _texDepth;

        private ROS2UnityComponent _ros2Unity;
        private ROS2Node _node;
        private IPublisher<sensor_msgs.msg.Image> _colorPub;
        private IPublisher<sensor_msgs.msg.Image> _depthPub;
        private IPublisher<sensor_msgs.msg.CameraInfo> _infoPub;

        private Shader _depthShader;
        private byte[] _baseRgb;
        private ushort[] _baseDepthMm;
        private byte[] _outRgb;
        private byte[] _outDepthBytes;
        private readonly object _frameLock = new object();

        private float _fx;
        private float _fy;
        private float _cx;
        private float _cy;

        private Stopwatch _pubWatch;
        private double _lastPubSec = 0.0;
        private bool _executableRegistered = false;

        private void Awake()
        {
            _cam = GetComponent<Camera>();
            _cam.enabled = false;
            _cam.depthTextureMode = DepthTextureMode.Depth;

            _rtColor = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            _rtDepth = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
            _texColor = new Texture2D(width, height, TextureFormat.RGB24, false);
            _texDepth = new Texture2D(width, height, TextureFormat.RFloat, false);

            _depthShader = Shader.Find("Hidden/RealSenseLinearDepth");
            _baseRgb = new byte[width * height * 3];
            _baseDepthMm = new ushort[width * height];
            _outRgb = new byte[width * height * 3];
            _outDepthBytes = new byte[width * height * 2];

            float vFovRad = _cam.fieldOfView * Mathf.Deg2Rad;
            _fy = (height * 0.5f) / Mathf.Tan(vFovRad * 0.5f);
            _fx = _fy;
            _cx = width * 0.5f;
            _cy = height * 0.5f;

            _pubWatch = Stopwatch.StartNew();
        }

        private void Start()
        {
            CaptureFromUnityCamera();
            InitROS2();
        }

        private void OnDestroy()
        {
            if (_ros2Unity != null && _executableRegistered)
            {
                _ros2Unity.UnregisterExecutable(OnBackgroundTick);
                _executableRegistered = false;
            }
            if (_rtColor != null) _rtColor.Release();
            if (_rtDepth != null) _rtDepth.Release();
        }

        private void InitROS2()
        {
            _ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                _node = _ros2Unity.CreateNode("realsense_sim");
                _colorPub = _node.CreatePublisher<sensor_msgs.msg.Image>("/camera/camera/color/image_raw");
                _depthPub = _node.CreatePublisher<sensor_msgs.msg.Image>("/camera/camera/depth/image_rect_raw");
                _infoPub = _node.CreatePublisher<sensor_msgs.msg.CameraInfo>("/camera/camera/depth/camera_info");
                if (!_executableRegistered)
                {
                    _ros2Unity.RegisterExecutable(OnBackgroundTick);
                    _executableRegistered = true;
                }
                Debug.Log("[RealSenseDepthSim] ROS2 Publishers initialized for RealSense color & depth (16UC1)");
            }
        }

        private void Update()
        {
            if (_ros2Unity != null && _ros2Unity.Ok() && _node == null)
            {
                InitROS2();
            }

            CaptureFromUnityCamera();
        }

        private void CaptureFromUnityCamera()
        {
            _cam.clearFlags = CameraClearFlags.Skybox;
            _cam.targetTexture = _rtColor;
            _cam.Render();
            RenderTexture.active = _rtColor;
            _texColor.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);

            if (_depthShader == null)
            {
                _depthShader = Shader.Find("Hidden/RealSenseLinearDepth");
            }

            _cam.clearFlags = CameraClearFlags.SolidColor;
            _cam.backgroundColor = Color.black;
            _cam.targetTexture = _rtDepth;
            if (_depthShader != null)
            {
                _cam.RenderWithShader(_depthShader, "");
            }
            else
            {
                _cam.Render();
            }
            RenderTexture.active = _rtDepth;
            _texDepth.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
            RenderTexture.active = null;
            _cam.targetTexture = null;

            NativeArray<byte> rawRgb = _texColor.GetRawTextureData<byte>();
            NativeArray<float> rawDepthMeters = _texDepth.GetRawTextureData<float>();
            int rowBytes = width * 3;

            lock (_frameLock)
            {
                for (int y = 0; y < height; y++)
                {
                    int srcRow = (height - 1 - y);
                    NativeArray<byte>.Copy(rawRgb, srcRow * rowBytes, _baseRgb, y * rowBytes, rowBytes);

                    int srcDepthOffset = srcRow * width;
                    int dstDepthOffset = y * width;
                    for (int x = 0; x < width; x++)
                    {
                        float dMeters = rawDepthMeters[srcDepthOffset + x];
                        ushort depthMm = 0;
                        if (dMeters > 0.1f && dMeters < 20.0f)
                        {
                            int mm = Mathf.RoundToInt(dMeters * 1000.0f);
                            depthMm = (ushort)Mathf.Clamp(mm, 0, 65535);
                        }
                        _baseDepthMm[dstDepthOffset + x] = depthMm;
                    }
                }
            }
        }

        private void OnBackgroundTick()
        {
            if (_colorPub == null || _depthPub == null || _infoPub == null) return;

            double now = _pubWatch.Elapsed.TotalSeconds;
            if (now - _lastPubSec < 1.0 / frameRate) return;
            _lastPubSec = now;

            LivoxMid360Sim.GetSimTime(out int sec, out uint nanosec);

            lock (_frameLock)
            {
                Buffer.BlockCopy(_baseRgb, 0, _outRgb, 0, _baseRgb.Length);
                for (int i = 0; i < _baseDepthMm.Length; i++)
                {
                    ushort mm = _baseDepthMm[i];
                    int bIdx = i * 2;
                    _outDepthBytes[bIdx] = (byte)(mm & 0xFF);
                    _outDepthBytes[bIdx + 1] = (byte)((mm >> 8) & 0xFF);
                }
            }

            if (SimpleClothShooter.IsClothFlying)
            {
                OverlayFlyingCloth(SimpleClothShooter.ActiveClothWorldPos);
            }

            sensor_msgs.msg.Image colorMsg = new sensor_msgs.msg.Image();
            colorMsg.Header.Frame_id = frameId;
            colorMsg.Header.Stamp.Sec = sec;
            colorMsg.Header.Stamp.Nanosec = nanosec;
            colorMsg.Width = (uint)width;
            colorMsg.Height = (uint)height;
            colorMsg.Encoding = "rgb8";
            colorMsg.Is_bigendian = 0;
            colorMsg.Step = (uint)(width * 3);
            colorMsg.Data = _outRgb;
            _colorPub.Publish(colorMsg);

            sensor_msgs.msg.Image depthMsg = new sensor_msgs.msg.Image();
            depthMsg.Header.Frame_id = frameId;
            depthMsg.Header.Stamp.Sec = sec;
            depthMsg.Header.Stamp.Nanosec = nanosec;
            depthMsg.Width = (uint)width;
            depthMsg.Height = (uint)height;
            depthMsg.Encoding = "16UC1";
            depthMsg.Is_bigendian = 0;
            depthMsg.Step = (uint)(width * 2);
            depthMsg.Data = _outDepthBytes;
            _depthPub.Publish(depthMsg);

            sensor_msgs.msg.CameraInfo infoMsg = new sensor_msgs.msg.CameraInfo();
            infoMsg.Header.Frame_id = frameId;
            infoMsg.Header.Stamp.Sec = sec;
            infoMsg.Header.Stamp.Nanosec = nanosec;
            infoMsg.Width = (uint)width;
            infoMsg.Height = (uint)height;
            infoMsg.Distortion_model = "plumb_bob";

            double[] kArr = new double[] { _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1 };
            Array.Copy(kArr, infoMsg.K, 9);
            double[] pArr = new double[] { _fx, 0, _cx, 0, 0, _fy, _cy, 0, 0, 0, 1, 0 };
            Array.Copy(pArr, infoMsg.P, 12);
            _infoPub.Publish(infoMsg);
        }

        private void OverlayFlyingCloth(Vector3 clothWorldPos)
        {
            Vector3 robotPos = KinematicOmniDrive.SharedRobotPosition;
            Quaternion r = KinematicOmniDrive.SharedRobotRotation;
            Quaternion invRobotRot = new Quaternion(-r.x, -r.y, -r.z, r.w);

            // Cloth position in robot local Unity coordinates (x=right, y=up, z=forward)
            Vector3 pRobotUnity = invRobotRot * (clothWorldPos - robotPos);
            // Convert to ROS base_link (x=forward, y=left, z=up)
            float bx = pRobotUnity.z;
            float by = -pRobotUnity.x;
            float bz = pRobotUnity.y;

            // Camera offset in tracker_params.yaml: x=0.50, y=0.0, z=1.00, pitch=20 deg (upward), roll=90 deg
            float dx = bx - 0.50f;
            float dy = by;
            float dz = bz - 1.00f;

            // Pitch up by 20 deg around camera left axis:
            float pitchRad = 20.0f * Mathf.Deg2Rad;
            float cosP = Mathf.Cos(pitchRad);
            float sinP = Mathf.Sin(pitchRad);

            // Forward distance along optical Z axis (exact inverse of cuda_kernels.cu lines 142-147)
            float zOpt = dx * cosP + dz * sinP;
            if (zOpt <= 0.25f || zOpt >= 5.45f) return;

            float xOpt = dx * sinP - dz * cosP;
            float yOpt = dy;

            int centerU = Mathf.RoundToInt(_fx * (xOpt / zOpt) + _cx);
            int centerV = Mathf.RoundToInt(_fy * (yOpt / zOpt) + _cy);

            int halfPatchU = Mathf.Clamp(Mathf.RoundToInt(_fx * (0.12f / zOpt)), 4, 80);
            int halfPatchV = Mathf.Clamp(Mathf.RoundToInt(_fy * (0.12f / zOpt)), 4, 80);

            ushort clothDepthMm = (ushort)Mathf.Clamp(Mathf.RoundToInt(zOpt * 1000.0f), 250, 15000);
            byte lo = (byte)(clothDepthMm & 0xFF);
            byte hi = (byte)((clothDepthMm >> 8) & 0xFF);

            int vMin = Mathf.Max(0, centerV - halfPatchV);
            int vMax = Mathf.Min(height - 1, centerV + halfPatchV);
            int uMin = Mathf.Max(0, centerU - halfPatchU);
            int uMax = Mathf.Min(width - 1, centerU + halfPatchU);

            for (int v = vMin; v <= vMax; v++)
            {
                for (int u = uMin; u <= uMax; u++)
                {
                    int pixIdx = v * width + u;
                    int dIdx = pixIdx * 2;
                    _outDepthBytes[dIdx] = lo;
                    _outDepthBytes[dIdx + 1] = hi;

                    int cIdx = pixIdx * 3;
                    _outRgb[cIdx] = 250;
                    _outRgb[cIdx + 1] = 250;
                    _outRgb[cIdx + 2] = 250;
                }
            }
        }
    }
}
