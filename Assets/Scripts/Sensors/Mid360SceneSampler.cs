using System;
using System.Collections.Generic;
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
        private sealed class ReadbackSlot
        {
            public ComputeBuffer Directions;
            public ComputeBuffer Ranges;
            public int Capacity;
            public AsyncGPUReadbackRequest Request;
            public bool Busy;
            public bool Finished;
            public Vector2[] Result;
            public Action<Vector2[]> Completed;
        }

        // Readback latency must not serialize acquisition into one scan every
        // two or three render frames. Each in-flight scan owns its GPU buffers.
        public const int MaxPendingReadbacks = 3;
        private readonly ReadbackSlot[] _slots = { new ReadbackSlot(), new ReadbackSlot(), new ReadbackSlot() };
        private readonly Queue<ReadbackSlot> _pending = new Queue<ReadbackSlot>();
        private int _kernel;
        private bool _disposed;
        private bool _warned;
        private int _captureFrame = -1;
        private Vector3 _captureOrigin;
        private float _captureMaxRange;
        private int _captureLayerMask;
        private RaycastHit[] _cpuHits = new RaycastHit[32];

        public bool UsingGpu { get; private set; }
        public bool IsBusy => !_disposed && (_pending.Count >= MaxPendingReadbacks || (!UsingGpu && _pending.Count > 0));
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

            ReadbackSlot slot = null;
            foreach (var candidate in _slots)
                if (!candidate.Busy) { slot = candidate; break; }
            if (slot == null) return false;

            try
            {
                EnsureBuffers(slot, directions.Length);
                slot.Directions.SetData(directions);
                CaptureDepth(origin, maxRange, layerMask);
                _compute.SetBuffer(_kernel, "_Directions", slot.Directions);
                _compute.SetBuffer(_kernel, "_Ranges", slot.Ranges);
                _compute.SetMatrixArray("_ViewProjections", _viewProjections);
                _compute.SetVector("_Origin", origin);
                _compute.SetInt("_Count", directions.Length);
                _compute.SetInt("_Resolution", Resolution);
                _compute.SetFloat("_MinRange", minRange);
                _compute.SetFloat("_MaxRange", maxRange);
                _compute.Dispatch(_kernel, (directions.Length + 63) / 64, 1, 1);
                slot.Busy = true;
                slot.Finished = false;
                slot.Completed = completed;
                _pending.Enqueue(slot);
                int count = directions.Length;
                slot.Request = AsyncGPUReadback.Request(slot.Ranges, count * sizeof(float) * 2, 0, request =>
                {
                    // Readback callbacks may arrive in a different order. Release
                    // buffers/deliver results only through the ordered queue below.
                    slot.Finished = true;
                    if (_disposed) return;
                    if (request.hasError)
                    {
                        UsingGpu = false;
                        WarnFallback("GPU readback failed");
                        return;
                    }
                    slot.Result = request.GetData<Vector2>().ToArray();
                });
                return true;
            }
            catch (Exception ex)
            {
                UsingGpu = false;
                WarnFallback(ex.Message);
                var result = ScanPhysics(origin, directions, minRange, maxRange, layerMask);
                if (!slot.Busy && _pending.Count == 0)
                {
                    completed(result);
                }
                else
                {
                    if (!slot.Busy)
                    {
                        slot.Busy = true;
                        _pending.Enqueue(slot);
                    }
                    slot.Completed = completed;
                    slot.Result = result;
                    slot.Finished = true;
                }
                return true;
            }
        }

        private void CaptureDepth(Vector3 origin, float maxRange, int layerMask)
        {
            // Catch-up scans use different Livox pattern segments, but observe
            // the same instantaneous scene/pose within this LateUpdate. Reuse
            // only their depth views, never another scan's measured ray results.
            if (Application.isPlaying && _captureFrame == Time.frameCount &&
                _captureOrigin.Equals(origin) && _captureMaxRange == maxRange && _captureLayerMask == layerMask)
                return;
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
                if (Application.isPlaying)
                {
                    _captureFrame = Time.frameCount;
                    _captureOrigin = origin;
                    _captureMaxRange = maxRange;
                    _captureLayerMask = layerMask;
                }
            }
            finally
            {
                _camera.targetTexture = null;
                RenderTexture.active = previousTarget;
            }
        }

        private static void EnsureBuffers(ReadbackSlot slot, int count)
        {
            if (slot.Capacity >= count) return;
            slot.Directions?.Release();
            slot.Ranges?.Release();
            slot.Capacity = Mathf.NextPowerOfTwo(count);
            slot.Directions = new ComputeBuffer(slot.Capacity, sizeof(float) * 3);
            slot.Ranges = new ComputeBuffer(slot.Capacity, sizeof(float) * 2);
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

        /// <summary>
        /// Refresh completion without waiting. Unity's automatic readback pump can
        /// run before the previous frame's GPU work completes, leaving IsBusy set
        /// for an extra frame even though that work is ready by LateUpdate.
        /// Call this on the main thread before deciding whether to start a scan.
        /// </summary>
        public void PollReadback()
        {
            if (_disposed) return;
            foreach (var slot in _slots)
                if (slot.Busy && !slot.Finished) slot.Request.Update();
            DeliverCompleted();
        }

        private void DeliverCompleted()
        {
            while (_pending.Count > 0 && _pending.Peek().Finished)
            {
                var slot = _pending.Dequeue();
                var completed = slot.Completed;
                var result = slot.Result;
                slot.Busy = false;
                slot.Completed = null;
                slot.Result = null;
                if (!_disposed && result != null) completed(result);
            }
        }

        /// <summary>Blocking diagnostics/shutdown only. Runtime capture never waits on the GPU.</summary>
        public void WaitForPendingScan()
        {
            foreach (var slot in _slots)
                if (slot.Busy && !slot.Finished) slot.Request.WaitForCompletion();
            DeliverCompleted();
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            WaitForPendingScan();
            ReleaseGpu();
        }

        private void ReleaseGpu()
        {
            UsingGpu = false;
            foreach (var slot in _slots)
            {
                slot.Directions?.Release();
                slot.Directions = null;
                slot.Ranges?.Release();
                slot.Ranges = null;
                slot.Busy = false;
                slot.Result = null;
                slot.Completed = null;
            }
            _pending.Clear();
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
