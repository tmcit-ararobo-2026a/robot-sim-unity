using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace RobotSim.Sensors
{
    /// <summary>
    /// Main-thread scene capture. Six radial-depth views cover the sphere; a compute
    /// shader samples only the requested Livox rays and asynchronously returns ranges.
    /// No CPU mesh copies, colliders, or hardware ray-tracing extensions are needed.
    /// </summary>
    public sealed class Mid360SceneSampler : IDisposable
    {
        private static readonly Vector3[] FaceForward =
        {
            Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back
        };
        private static readonly Vector3[] FaceUp =
        {
            Vector3.up, Vector3.up, Vector3.back, Vector3.forward, Vector3.up, Vector3.up
        };

        private Camera _camera;
        private Shader _depthShader;
        private ComputeShader _compute;
        private readonly RenderTexture[] _faces = new RenderTexture[6];
        private readonly Matrix4x4[] _viewProjections = new Matrix4x4[6];
        private ComputeBuffer _directions;
        private ComputeBuffer _ranges;
        private AsyncGPUReadbackRequest _readback;
        private int _capacity;
        private int _kernel;
        private bool _disposed;
        private bool _warned;
        private RaycastHit[] _cpuHits = new RaycastHit[32];

        public bool UsingGpu { get; private set; }
        public bool IsBusy { get; private set; }
        public int Resolution { get; }

        public Mid360SceneSampler(bool preferGpu, int resolution)
        {
            Resolution = Mathf.Clamp(Mathf.ClosestPowerOfTwo(resolution), 64, 2048);
            if (!preferGpu) return;
            if (!SystemInfo.supportsComputeShaders || !SystemInfo.supportsAsyncGPUReadback ||
                !SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.RGFloat) ||
                GraphicsSettings.currentRenderPipeline != null)
            {
                WarnFallback("Compute/async readback/RGFloat or Built-in Render Pipeline unavailable");
                return;
            }

            try
            {
                _depthShader = Resources.Load<Shader>("Mid360/Mid360RadialDepth");
                var source = Resources.Load<ComputeShader>("Mid360/Mid360SampleRanges");
                if (_depthShader == null || !_depthShader.isSupported || source == null)
                    throw new InvalidOperationException("MID360 GPU shader assets unavailable");
                _compute = UnityEngine.Object.Instantiate(source);
                _kernel = _compute.FindKernel("SampleRanges");
                var cameraObject = new GameObject("MID360 Depth Capture") { hideFlags = HideFlags.HideAndDontSave };
                _camera = cameraObject.AddComponent<Camera>();
                _camera.enabled = false;
                _camera.clearFlags = CameraClearFlags.SolidColor;
                _camera.backgroundColor = Color.black;
                _camera.fieldOfView = 90f;
                _camera.aspect = 1f;
                _camera.nearClipPlane = 0.001f;
                _camera.allowHDR = false;
                _camera.allowMSAA = false;
                _camera.useOcclusionCulling = false;
                _camera.renderingPath = RenderingPath.Forward;
                for (int face = 0; face < 6; face++)
                {
                    _faces[face] = new RenderTexture(Resolution, Resolution, 24, RenderTextureFormat.RGFloat,
                        RenderTextureReadWrite.Linear)
                    {
                        name = "MID360 Depth " + face,
                        filterMode = FilterMode.Point,
                        wrapMode = TextureWrapMode.Clamp,
                        hideFlags = HideFlags.HideAndDontSave
                    };
                    if (!_faces[face].Create()) throw new InvalidOperationException("Depth texture allocation failed");
                    _compute.SetTexture(_kernel, "_Depth" + face, _faces[face]);
                }
                UsingGpu = true;
            }
            catch (Exception ex)
            {
                ReleaseGpu();
                WarnFallback(ex.Message);
            }
        }

        // Directions must be normalized Unity world vectors. Callback owns the returned array.
        public bool TryScan(Vector3 origin, Vector3[] directions, float minRange, float maxRange,
            int layerMask, Action<Vector2[]> completed)
        {
            if (_disposed || IsBusy) return false;
            if (!UsingGpu)
            {
                completed(ScanPhysics(origin, directions, minRange, maxRange, layerMask));
                return true;
            }

            try
            {
                EnsureBuffers(directions.Length);
                _directions.SetData(directions);
                CaptureDepth(origin, maxRange, layerMask);
                _compute.SetMatrixArray("_ViewProjections", _viewProjections);
                _compute.SetVector("_Origin", origin);
                _compute.SetInt("_Count", directions.Length);
                _compute.SetInt("_Resolution", Resolution);
                _compute.SetFloat("_MinRange", minRange);
                _compute.SetFloat("_MaxRange", maxRange);
                _compute.Dispatch(_kernel, (directions.Length + 63) / 64, 1, 1);
                IsBusy = true;
                int count = directions.Length;
                _readback = AsyncGPUReadback.Request(_ranges, count * sizeof(float) * 2, 0, request =>
                {
                    IsBusy = false;
                    if (_disposed) return;
                    if (request.hasError)
                    {
                        UsingGpu = false;
                        WarnFallback("GPU readback failed");
                        return;
                    }
                    completed(request.GetData<Vector2>().ToArray());
                });
                return true;
            }
            catch (Exception ex)
            {
                IsBusy = false;
                UsingGpu = false;
                WarnFallback(ex.Message);
                completed(ScanPhysics(origin, directions, minRange, maxRange, layerMask));
                return true;
            }
        }

        private void CaptureDepth(Vector3 origin, float maxRange, int layerMask)
        {
            RenderTexture previousTarget = RenderTexture.active;
            try
            {
                _camera.transform.position = origin;
                _camera.cullingMask = layerMask;
                _camera.farClipPlane = maxRange;
                for (int face = 0; face < 6; face++)
                {
                    _camera.transform.rotation = Quaternion.LookRotation(FaceForward[face], FaceUp[face]);
                    _camera.targetTexture = _faces[face];
                    _viewProjections[face] = GL.GetGPUProjectionMatrix(_camera.projectionMatrix, true) * _camera.worldToCameraMatrix;
                    _camera.RenderWithShader(_depthShader, "");
                }
            }
            finally
            {
                _camera.targetTexture = null;
                RenderTexture.active = previousTarget;
            }
        }

        private void EnsureBuffers(int count)
        {
            if (_capacity >= count) return;
            _directions?.Release();
            _ranges?.Release();
            _capacity = Mathf.NextPowerOfTwo(count);
            _directions = new ComputeBuffer(_capacity, sizeof(float) * 3);
            _ranges = new ComputeBuffer(_capacity, sizeof(float) * 2);
            _compute.SetBuffer(_kernel, "_Directions", _directions);
            _compute.SetBuffer(_kernel, "_Ranges", _ranges);
        }

        private Vector2[] ScanPhysics(Vector3 origin, Vector3[] directions, float minRange, float maxRange,
            int layerMask)
        {
            Physics.SyncTransforms();
            var ranges = new Vector2[directions.Length];
            for (int i = 0; i < directions.Length; i++)
            {
                int hits;
                // RaycastNonAlloc is unordered; grow instead of losing nearer hits in dense scenes.
                while (true)
                {
                    hits = Physics.RaycastNonAlloc(origin, directions[i], _cpuHits, maxRange,
                        layerMask, QueryTriggerInteraction.Ignore);
                    if (hits < _cpuHits.Length) break;
                    Array.Resize(ref _cpuHits, _cpuHits.Length * 2);
                }
                float nearest = float.PositiveInfinity;
                float reflectivity = 0f;
                for (int h = 0; h < hits; h++)
                {
                    if (_cpuHits[h].distance >= nearest) continue;
                    nearest = _cpuHits[h].distance;
                    var surface = _cpuHits[h].collider.GetComponent<Mid360Surface>();
                    reflectivity = surface != null && surface.isActiveAndEnabled ? surface.reflectivity : 0f;
                }
                ranges[i] = nearest >= minRange && nearest <= maxRange ? new Vector2(nearest, reflectivity) : new Vector2(-1, 0);
            }
            return ranges;
        }

        private void WarnFallback(string reason)
        {
            if (_warned) return;
            _warned = true;
            Debug.LogWarning("[LivoxMid360Sim] " + reason + "; using CPU physics (Collider required).");
        }

        /// <summary>Blocking diagnostics/shutdown only. Runtime capture never waits on the GPU.</summary>
        public void WaitForPendingScan()
        {
            if (IsBusy) _readback.WaitForCompletion();
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            WaitForPendingScan();
            IsBusy = false;
            ReleaseGpu();
        }

        private void ReleaseGpu()
        {
            UsingGpu = false;
            _directions?.Release();
            _directions = null;
            _ranges?.Release();
            _ranges = null;
            foreach (var face in _faces)
            {
                if (face == null) continue;
                face.Release();
                DestroyObject(face);
            }
            if (_camera != null) DestroyObject(_camera.gameObject);
            if (_compute != null) DestroyObject(_compute);
        }

        private static void DestroyObject(UnityEngine.Object obj)
        {
            if (Application.isPlaying) UnityEngine.Object.Destroy(obj);
            else UnityEngine.Object.DestroyImmediate(obj);
        }
    }
}
