using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace RobotSim.Sensors
{
    /// <summary>Two-level triangle BVH, independent of Colliders and depth pixels.
    /// Each ray carries its physics-interval fraction. Mesh geometry and instance
    /// poses are immutable snapshots until their ordered readback completes.</summary>
    public sealed class Mid360TemporalScene : IDisposable
    {
        // float4-only GPU records keep the same layout on OpenGL, Vulkan and DX.
        public struct TimedRay { public Vector4 Origin, Direction; }
        private struct Node { public Vector4 Min, Max, Links; }
        private struct Triangle { public Vector4 A, B, C, PreviousA, PreviousB, PreviousC; }
        private struct Primitive { public Triangle Triangle; public Bounds Bounds; }
        private struct Instance
        {
            public Vector4 PreviousPosition, Position, PreviousRotation, Rotation, PreviousScale, Scale;
        }
        private sealed class Geometry
        {
            public Triangle[] Triangles;
            public Node[] Nodes;
            public Bounds Bounds;
            public int Root;
        }
        private sealed class History
        {
            public Vector3 Position, Scale;
            public Quaternion Rotation;
            public Mesh Bake;
            public Vector3[] Vertices;
            public Geometry Geometry;
        }
        private struct Surface
        {
            public Instance Instance;
            public Geometry Geometry;
            public Bounds Bounds;
        }
        private sealed class Slot
        {
            public ComputeBuffer Rays, Ranges, Instances, Top;
            public AsyncGPUReadbackRequest Request;
            public bool Busy;
            public Action<Vector2[]> Completed;
            public Func<Vector2[]> Fallback;
        }

        private readonly Dictionary<int, Geometry> _meshes = new Dictionary<int, Geometry>();
        private readonly Dictionary<int, History> _history = new Dictionary<int, History>();
        private readonly List<Geometry> _geometry = new List<Geometry>();
        private readonly List<Surface> _surfaces = new List<Surface>();
        private readonly List<Node> _top = new List<Node>();
        private readonly List<Slot> _slots = new List<Slot>();
        private readonly Queue<Slot> _pending = new Queue<Slot>();
        private struct Retired { public ComputeBuffer Buffer; public AsyncGPUReadbackRequest Fence; }
        private readonly List<Retired> _retired = new List<Retired>();
        private readonly HashSet<int> _seen = new HashSet<int>();
        private readonly List<int> _removed = new List<int>();
        private Surface[] _snapshot = Array.Empty<Surface>();
        private Node[] _topSnapshot = Array.Empty<Node>();
        private ComputeShader _shader;
        private ComputeBuffer _triangles, _nodes;
        private int _kernel;
        private bool _geometryDirty, _disposed, _warned;
        private readonly int _layerMask;
        public bool UsingGpu { get; private set; }
        public int InstanceCount => _snapshot.Length;
        public int TriangleCount { get; private set; }
        public int PendingCount => _pending.Count;

        public Mid360TemporalScene(bool preferGpu = false, int layerMask = ~0)
        {
            _layerMask = layerMask;
            if (!preferGpu) return;
            if (!SystemInfo.supportsComputeShaders || !SystemInfo.supportsAsyncGPUReadback)
            {
                FallbackWarning("Compute shaders or async GPU readback unavailable");
                return;
            }
            var asset = Resources.Load<ComputeShader>("Mid360/Mid360MeshRanges");
            if (asset == null) throw new InvalidOperationException("MID360 mesh compute shader is missing");
            _shader = UnityEngine.Object.Instantiate(asset);
            _kernel = _shader.FindKernel("SampleMeshRanges");
            UsingGpu = true;
        }

        public void Capture()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(Mid360TemporalScene));
            PollReadback();
            _surfaces.Clear();
            _seen.Clear();
            foreach (var renderer in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsSortMode.None))
            {
                if (!renderer.enabled || renderer.forceRenderingOff || !renderer.gameObject.activeInHierarchy ||
                    (_layerMask & (1 << renderer.gameObject.layer)) == 0) continue;
                var filter = renderer.GetComponent<MeshFilter>();
                var skin = renderer as SkinnedMeshRenderer;
                Mesh mesh = skin != null ? skin.sharedMesh : filter != null ? filter.sharedMesh : null;
                if (mesh == null) continue; // Lights, lines and particles are not triangle surfaces.
                int id = renderer.GetInstanceID();
                _seen.Add(id);
                Transform transform = renderer.transform;
                Vector3 position = transform.position;
                Quaternion rotation = transform.rotation;
                Vector3 scale = transform.lossyScale;
                // Ignore visual Rigidbody interpolation: measurement uses physics poses.
                var body = renderer.GetComponentInParent<Rigidbody>();
                if (body != null)
                {
                    Quaternion delta = body.rotation * Quaternion.Inverse(body.transform.rotation);
                    position = body.position + delta * (position - body.transform.position);
                    rotation = delta * rotation;
                }
                if (!_history.TryGetValue(id, out var previous))
                {
                    previous = new History { Position = position, Rotation = rotation, Scale = scale };
                    _history.Add(id, previous);
                }
                Geometry geometry;
                if (skin != null)
                {
                    if (previous.Bake == null) previous.Bake = new Mesh { name = "MID360 skinned snapshot", hideFlags = HideFlags.HideAndDontSave };
                    skin.BakeMesh(previous.Bake);
                    Vector3[] vertices = previous.Bake.vertices;
                    geometry = BuildGeometry(previous.Bake, vertices, previous.Vertices);
                    previous.Vertices = vertices;
                    if (previous.Geometry != null) _geometry.Remove(previous.Geometry);
                    _geometry.Add(geometry);
                    previous.Geometry = geometry;
                    _geometryDirty = true;
                }
                else if (!_meshes.TryGetValue(mesh.GetInstanceID(), out geometry))
                {
                    if (!mesh.isReadable)
                        throw new InvalidOperationException("MID360 needs readable triangle data: enable Read/Write on mesh '" + mesh.name + "'. Collider substitution is disabled.");
                    geometry = BuildGeometry(mesh, mesh.vertices, null);
                    _meshes.Add(mesh.GetInstanceID(), geometry);
                    _geometry.Add(geometry);
                    _geometryDirty = true;
                }
                if (geometry.Triangles.Length == 0) continue;
                var material = renderer.GetComponent<Mid360Surface>();
                float reflectivity = material != null && material.isActiveAndEnabled ? material.reflectivity : 0;
                var instance = new Instance
                {
                    PreviousPosition = V(previous.Position, reflectivity), Position = V(position, 0),
                    PreviousRotation = Q(previous.Rotation), Rotation = Q(rotation),
                    PreviousScale = V(previous.Scale, 0), Scale = V(scale, 0)
                };
                Bounds bounds;
                if (Quaternion.Angle(previous.Rotation, rotation) < 0.00001f && previous.Scale == scale)
                {
                    // Tight bounds for static/translating field meshes, whose local
                    // vertices can be far from their transform's origin.
                    bounds = TransformBounds(geometry.Bounds, position, rotation, scale);
                    bounds.Encapsulate(TransformBounds(geometry.Bounds, previous.Position, previous.Rotation, previous.Scale));
                }
                else
                {
                    float largestScale = Mathf.Max(MaxAbs(scale), MaxAbs(previous.Scale));
                    float radius = (geometry.Bounds.center.magnitude + geometry.Bounds.extents.magnitude) * largestScale;
                    bounds = new Bounds(position, Vector3.one * (2 * radius + 0.0001f));
                    bounds.Encapsulate(new Bounds(previous.Position, Vector3.one * (2 * radius + 0.0001f)));
                }
                _surfaces.Add(new Surface { Instance = instance, Geometry = geometry, Bounds = bounds });
                previous.Position = position;
                previous.Rotation = rotation;
                previous.Scale = scale;
            }
            _removed.Clear();
            foreach (int id in _history.Keys) if (!_seen.Contains(id)) _removed.Add(id);
            foreach (int id in _removed)
            {
                var old = _history[id];
                if (old.Bake != null)
                {
                    UnityEngine.Object.DestroyImmediate(old.Bake);
                    _geometry.Remove(old.Geometry);
                    _geometryDirty = true;
                }
                _history.Remove(id);
            }
            if (_geometryDirty || (UsingGpu && _nodes == null)) UploadGeometry();
            _top.Clear();
            if (_surfaces.Count > 0) BuildTop(0, _surfaces.Count);
            _snapshot = _surfaces.ToArray();
            _topSnapshot = _top.ToArray();
        }

        private static Bounds TransformBounds(Bounds local, Vector3 position, Quaternion rotation, Vector3 scale)
        {
            var result = new Bounds(position + rotation * Vector3.Scale(local.center, scale), Vector3.zero);
            for (int i = 0; i < 8; i++)
            {
                Vector3 corner = local.center + Vector3.Scale(local.extents,
                    new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                result.Encapsulate(position + rotation * Vector3.Scale(corner, scale));
            }
            return result;
        }

        private static Geometry BuildGeometry(Mesh mesh, Vector3[] vertices, Vector3[] previous)
        {
            if (previous == null || previous.Length != vertices.Length) previous = vertices;
            var primitives = new List<Primitive>();
            Bounds bounds = new Bounds();
            bool first = true;
            for (int sub = 0; sub < mesh.subMeshCount; sub++)
            {
                if (mesh.GetTopology(sub) != MeshTopology.Triangles) continue;
                int[] indices = mesh.GetTriangles(sub);
                for (int i = 0; i < indices.Length; i += 3)
                {
                    int a = indices[i], b = indices[i + 1], c = indices[i + 2];
                    var triangle = new Triangle
                    {
                        A = V(vertices[a], 0), B = V(vertices[b], 0), C = V(vertices[c], 0),
                        PreviousA = V(previous[a], 0), PreviousB = V(previous[b], 0), PreviousC = V(previous[c], 0)
                    };
                    Bounds tb = TriangleBounds(triangle);
                    if (first) { bounds = tb; first = false; } else bounds.Encapsulate(tb);
                    primitives.Add(new Primitive { Triangle = triangle, Bounds = tb });
                }
            }
            var nodes = new List<Node>();
            Primitive[] ordered = primitives.ToArray();
            if (ordered.Length > 0) BuildMesh(nodes, ordered, 0, ordered.Length);
            var triangles = new Triangle[ordered.Length];
            for (int i = 0; i < triangles.Length; i++) triangles[i] = ordered[i].Triangle;
            return new Geometry { Triangles = triangles, Nodes = nodes.ToArray(), Bounds = bounds };
        }

        private static int Axis(Vector3 size) => size.x >= size.y && size.x >= size.z ? 0 : size.y >= size.z ? 1 : 2;
        private static Bounds TriangleBounds(Triangle t)
        {
            var bounds = new Bounds((Vector3)t.A, Vector3.zero);
            bounds.Encapsulate((Vector3)t.B); bounds.Encapsulate((Vector3)t.C);
            bounds.Encapsulate((Vector3)t.PreviousA); bounds.Encapsulate((Vector3)t.PreviousB); bounds.Encapsulate((Vector3)t.PreviousC);
            return bounds;
        }
        private static void BuildMesh(List<Node> nodes, Primitive[] triangles, int start, int count)
        {
            Bounds bounds = triangles[start].Bounds;
            for (int i = 1; i < count; i++) bounds.Encapsulate(triangles[start + i].Bounds);
            int index = nodes.Count;
            nodes.Add(default);
            if (count > 4)
            {
                int axis = Axis(bounds.size);
                int half = count / 2;
                Partition(triangles, start, start + count - 1, start + half, axis);
                BuildMesh(nodes, triangles, start, half);
                BuildMesh(nodes, triangles, start + half, count - half);
            }
            nodes[index] = MakeNode(bounds, start, count <= 4 ? count : 0, nodes.Count);
        }
        // Median selection avoids re-sorting a million-triangle CAD mesh at
        // every tree level. Bounds are calculated once per primitive, not once
        // per comparator invocation.
        private static void Partition(Primitive[] primitives, int low, int high, int median, int axis)
        {
            while (low < high)
            {
                float pivot = primitives[low + (high - low) / 2].Bounds.center[axis];
                int i = low, j = high;
                while (i <= j)
                {
                    while (primitives[i].Bounds.center[axis] < pivot) i++;
                    while (primitives[j].Bounds.center[axis] > pivot) j--;
                    if (i > j) break;
                    Primitive swap = primitives[i]; primitives[i] = primitives[j]; primitives[j] = swap;
                    i++; j--;
                }
                if (median <= j) high = j;
                else if (median >= i) low = i;
                else break;
            }
        }

        private void BuildTop(int start, int count)
        {
            Bounds bounds = _surfaces[start].Bounds;
            for (int i = 1; i < count; i++) bounds.Encapsulate(_surfaces[start + i].Bounds);
            int index = _top.Count;
            _top.Add(default);
            if (count > 1)
            {
                int axis = Axis(bounds.size);
                _surfaces.Sort(start, count, Comparer<Surface>.Create((a, b) => a.Bounds.center[axis].CompareTo(b.Bounds.center[axis])));
                int half = count / 2;
                BuildTop(start, half); BuildTop(start + half, count - half);
            }
            _top[index] = MakeNode(bounds, start, count == 1 ? 1 : 0, _top.Count);
        }
        private static Node MakeNode(Bounds bounds, int first, int count, int escape) => new Node
        {
            Min = V(bounds.min - Vector3.one * 0.00001f, first), Max = V(bounds.max + Vector3.one * 0.00001f, count), Links = new Vector4(escape, 0, 0, 0)
        };
        private static Vector4 V(Vector3 v, float w) => new Vector4(v.x, v.y, v.z, w);
        private static Vector4 Q(Quaternion q) => new Vector4(q.x, q.y, q.z, q.w);
        private static Quaternion Q(Vector4 q) => new Quaternion(q.x, q.y, q.z, q.w);
        private static float MaxAbs(Vector3 v) => Mathf.Max(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));

        private void UploadGeometry()
        {
            var triangles = new List<Triangle>();
            var nodes = new List<Node>();
            foreach (var geometry in _geometry)
            {
                int triangleOffset = triangles.Count, nodeOffset = nodes.Count;
                geometry.Root = nodeOffset;
                triangles.AddRange(geometry.Triangles);
                foreach (var source in geometry.Nodes)
                {
                    Node node = source;
                    node.Min.w += triangleOffset;
                    node.Links.x += nodeOffset;
                    nodes.Add(node);
                }
            }
            TriangleCount = triangles.Count;
            if (UsingGpu)
            {
                // Do not release a buffer still referenced by submitted GPU work.
                Retire(_triangles);
                Retire(_nodes);
                _triangles = new ComputeBuffer(Math.Max(1, triangles.Count), 96);
                _nodes = new ComputeBuffer(Math.Max(1, nodes.Count), 48);
                if (triangles.Count > 0) _triangles.SetData(triangles);
                if (nodes.Count > 0) _nodes.SetData(nodes);
            }
            _geometryDirty = false;
        }

        public void Submit(TimedRay[] rays, float minRange, float maxRange, Action<Vector2[]> completed)
        {
            if (_disposed) throw new ObjectDisposedException(nameof(Mid360TemporalScene));
            if (rays.Length == 0) { completed(Array.Empty<Vector2>()); return; }
            Surface[] surfaces = _snapshot;
            Node[] top = _topSnapshot;
            Func<Vector2[]> fallback = () => SampleAll(surfaces, top, rays, minRange, maxRange);
            if (!UsingGpu)
            {
                // Preserve order if GPU failure leaves older acquisitions in flight.
                WaitForPending();
                completed(fallback());
                return;
            }
            PollReadback();
            Slot slot = _slots.Find(s => !s.Busy);
            if (slot == null)
            {
                // Bound catch-up memory without dropping or resampling old intervals.
                if (_slots.Count >= 64) { _pending.Peek().Request.WaitForCompletion(); PollReadback(); slot = _slots.Find(s => !s.Busy); }
                if (slot == null) { slot = new Slot(); _slots.Add(slot); }
            }
            try
            {
                Ensure(ref slot.Rays, rays.Length, 32);
                Ensure(ref slot.Ranges, rays.Length, 8);
                Ensure(ref slot.Instances, surfaces.Length, 96);
                Ensure(ref slot.Top, top.Length, 48);
                var instances = new Instance[surfaces.Length];
                for (int i = 0; i < surfaces.Length; i++)
                {
                    instances[i] = surfaces[i].Instance;
                    instances[i].Position.w = surfaces[i].Geometry.Root;
                }
                slot.Rays.SetData(rays);
                if (top.Length > 0) slot.Top.SetData(top);
                if (instances.Length > 0) slot.Instances.SetData(instances);
                _shader.SetBuffer(_kernel, "_Rays", slot.Rays);
                _shader.SetBuffer(_kernel, "_Ranges", slot.Ranges);
                _shader.SetBuffer(_kernel, "_Instances", slot.Instances);
                _shader.SetBuffer(_kernel, "_TopNodes", slot.Top);
                _shader.SetBuffer(_kernel, "_MeshNodes", _nodes);
                _shader.SetBuffer(_kernel, "_Triangles", _triangles);
                _shader.SetInt("_Count", rays.Length);
                _shader.SetInt("_TopCount", top.Length);
                _shader.SetFloat("_MinRange", minRange);
                _shader.SetFloat("_MaxRange", maxRange);
                _shader.Dispatch(_kernel, (rays.Length + 63) / 64, 1, 1);
                slot.Completed = completed;
                slot.Fallback = fallback;
                slot.Request = AsyncGPUReadback.Request(slot.Ranges, rays.Length * 8, 0);
                slot.Busy = true;
                _pending.Enqueue(slot);
            }
            catch (Exception ex)
            {
                UsingGpu = false;
                FallbackWarning(ex.Message);
                WaitForPending();
                completed(fallback());
            }
        }
        private static void Ensure(ref ComputeBuffer buffer, int count, int stride)
        {
            count = Math.Max(1, count);
            if (buffer != null && buffer.count >= count) return;
            buffer?.Release();
            buffer = new ComputeBuffer(Mathf.NextPowerOfTwo(count), stride);
        }
        public void PollReadback()
        {
            foreach (var slot in _pending) if (!slot.Request.done) slot.Request.Update();
            while (_pending.Count > 0 && _pending.Peek().Request.done)
            {
                Slot slot = _pending.Dequeue();
                Vector2[] result;
                if (slot.Request.hasError)
                {
                    UsingGpu = false;
                    FallbackWarning("GPU readback failed; replaying the saved mesh/pose snapshot");
                    result = slot.Fallback();
                }
                else result = slot.Request.GetData<Vector2>().ToArray();
                var callback = slot.Completed;
                slot.Busy = false; slot.Completed = null; slot.Fallback = null;
                if (!_disposed) callback(result);
            }
            for (int i = _retired.Count - 1; i >= 0; i--)
                if (_retired[i].Fence.done)
                {
                    _retired[i].Buffer.Release();
                    _retired.RemoveAt(i);
                }
        }
        private void Retire(ComputeBuffer buffer)
        {
            if (buffer == null) return;
            if (_pending.Count == 0) { buffer.Release(); return; }
            Slot last = null;
            foreach (var slot in _pending) last = slot;
            _retired.Add(new Retired { Buffer = buffer, Fence = last.Request });
        }
        public void WaitForPending()
        {
            foreach (var slot in _pending) slot.Request.WaitForCompletion();
            PollReadback();
        }
        private void FallbackWarning(string reason)
        {
            if (_warned) return;
            _warned = true;
            Debug.LogWarning("[LivoxMid360Sim] " + reason + "; using CPU triangle BVH (no Collider substitution).");
        }

        // CPU reference/fallback uses exactly the same triangle/pose snapshots.
        public Vector2 Sample(Vector3 origin, Vector3 direction, float fraction, float minRange, float maxRange) =>
            Trace(_snapshot, _topSnapshot, new TimedRay { Origin = V(origin, fraction), Direction = V(direction, 0) }, minRange, maxRange);
        private static Vector2[] SampleAll(Surface[] surfaces, Node[] top, TimedRay[] rays, float min, float max)
        {
            var result = new Vector2[rays.Length];
            for (int i = 0; i < result.Length; i++) result[i] = Trace(surfaces, top, rays[i], min, max);
            return result;
        }
        private static Vector2 Trace(Surface[] surfaces, Node[] top, TimedRay ray, float min, float max)
        {
            float nearest = max, reflectivity = 0;
            bool found = false;
            Vector3 origin = ray.Origin, direction = ray.Direction;
            float fraction = Mathf.Clamp01(ray.Origin.w);
            int n = 0;
            while (n < top.Length)
            {
                Node node = top[n];
                if (!Intersects(node, origin, direction, nearest)) { n = (int)node.Links.x; continue; }
                if (node.Max.w == 0) { n++; continue; }
                Surface surface = surfaces[(int)node.Min.w];
                Instance instance = surface.Instance;
                Vector3 position = Vector3.Lerp(instance.PreviousPosition, instance.Position, fraction);
                Quaternion inverse = Quaternion.Inverse(Quaternion.Slerp(Q(instance.PreviousRotation), Q(instance.Rotation), fraction));
                Vector3 scale = Vector3.Lerp(instance.PreviousScale, instance.Scale, fraction);
                if (Mathf.Abs(scale.x * scale.y * scale.z) > 1e-12f)
                {
                    Vector3 o = inverse * (origin - position), d = inverse * direction;
                    o = new Vector3(o.x / scale.x, o.y / scale.y, o.z / scale.z);
                    d = new Vector3(d.x / scale.x, d.y / scale.y, d.z / scale.z); // Do not normalize: t stays world metres.
                    Geometry geometry = surface.Geometry;
                    int m = 0;
                    while (m < geometry.Nodes.Length)
                    {
                        Node meshNode = geometry.Nodes[m];
                        if (!Intersects(meshNode, o, d, nearest)) { m = (int)meshNode.Links.x; continue; }
                        if (meshNode.Max.w == 0) { m++; continue; }
                        int first = (int)meshNode.Min.w, count = (int)meshNode.Max.w;
                        for (int t = first; t < first + count; t++)
                        {
                            Triangle triangle = geometry.Triangles[t];
                            Vector3 a = Vector3.Lerp(triangle.PreviousA, triangle.A, fraction);
                            Vector3 b = Vector3.Lerp(triangle.PreviousB, triangle.B, fraction);
                            Vector3 c = Vector3.Lerp(triangle.PreviousC, triangle.C, fraction);
                            if (!HitTriangle(o, d, a, b, c, nearest, out float distance)) continue;
                            nearest = distance; reflectivity = instance.PreviousPosition.w; found = true;
                        }
                        m = (int)meshNode.Links.x;
                    }
                }
                n = (int)node.Links.x;
            }
            return found && nearest >= min ? new Vector2(nearest, reflectivity) : new Vector2(-1, 0);
        }
        private static bool Intersects(Node node, Vector3 origin, Vector3 direction, float limit)
        {
            float low = 0, high = limit;
            for (int axis = 0; axis < 3; axis++)
            {
                if (Mathf.Abs(direction[axis]) < 1e-20f)
                {
                    if (origin[axis] < node.Min[axis] || origin[axis] > node.Max[axis]) return false;
                    continue;
                }
                float a = (node.Min[axis] - origin[axis]) / direction[axis];
                float b = (node.Max[axis] - origin[axis]) / direction[axis];
                low = Mathf.Max(low, Mathf.Min(a, b)); high = Mathf.Min(high, Mathf.Max(a, b));
                if (low > high) return false;
            }
            return true;
        }
        private static bool HitTriangle(Vector3 origin, Vector3 direction, Vector3 a, Vector3 b, Vector3 c, float limit, out float distance)
        {
            distance = 0;
            Vector3 edge1 = b - a, edge2 = c - a, cross = Vector3.Cross(direction, edge2);
            float det = Vector3.Dot(edge1, cross);
            if (Mathf.Abs(det) < 1e-9f) return false;
            float inv = 1 / det;
            Vector3 delta = origin - a;
            float u = Vector3.Dot(delta, cross) * inv;
            if (u < 0 || u > 1) return false;
            Vector3 q = Vector3.Cross(delta, edge1);
            float v = Vector3.Dot(direction, q) * inv;
            if (v < 0 || u + v > 1) return false;
            distance = Vector3.Dot(edge2, q) * inv;
            return distance >= 0 && distance <= limit;
        }
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            WaitForPending();
            foreach (var slot in _slots) { slot.Rays?.Release(); slot.Ranges?.Release(); slot.Instances?.Release(); slot.Top?.Release(); }
            _triangles?.Release(); _nodes?.Release();
            foreach (var previous in _history.Values) if (previous.Bake != null) UnityEngine.Object.DestroyImmediate(previous.Bake);
            if (_shader != null) UnityEngine.Object.DestroyImmediate(_shader);
        }
    }
}
