using System.Collections.Generic;
using UnityEngine;

namespace RobotSim.Environment
{
    // CAD paint/tile strips are 1 mm solids. Their vertical edges must not catch
    // sliding furniture or the chassis; collision uses one seamless floor instead.
    [RequireComponent(typeof(MeshCollider))]
    public class FieldGroundPhysics : MonoBehaviour
    {
        private Mesh _originalMesh;
        private Mesh _collisionMesh;
        private MeshCollider _collider;
        private BoxCollider _centralStep;
        public float FloorHeight { get; private set; }

        private void Awake()
        {
            _collider = GetComponent<MeshCollider>();
            _originalMesh = _collider.sharedMesh;
            if (_originalMesh == null) return;
            Vector3[] vertices = _originalMesh.vertices;
            FloorHeight = _originalMesh.bounds.min.y + 0.001f;
            int[] originalTriangles = _originalMesh.triangles;
            bool reverseWinding = HasInwardTopFaces(vertices, originalTriangles, _originalMesh.bounds.max.y);
            bool hasCentralStep = TryGetCentralStepBounds(vertices, originalTriangles, _originalMesh.bounds,
                out Bounds stepBounds);
            Bounds stepTriangleBounds = stepBounds;
            stepTriangleBounds.Expand(0.0002f);
            var structureTriangles = new List<int>();
            for (int i = 0; i < originalTriangles.Length; i += 3)
            {
                float top = Mathf.Max(vertices[originalTriangles[i]].y,
                    Mathf.Max(vertices[originalTriangles[i + 1]].y, vertices[originalTriangles[i + 2]].y));
                // Retain real walls, boundary fences and the central platform.
                if (top <= FloorHeight + 0.002f) continue;
                // Use a single solid cuboid for the highest rectangular platform.
                // Triangle edges on its top otherwise produce upward snag impulses.
                if (hasCentralStep && stepTriangleBounds.Contains(vertices[originalTriangles[i]]) &&
                    stepTriangleBounds.Contains(vertices[originalTriangles[i + 1]]) &&
                    stepTriangleBounds.Contains(vertices[originalTriangles[i + 2]])) continue;
                structureTriangles.Add(originalTriangles[i]);
                structureTriangles.Add(originalTriangles[i + (reverseWinding ? 2 : 1)]);
                structureTriangles.Add(originalTriangles[i + (reverseWinding ? 1 : 2)]);
            }
            _collisionMesh = new Mesh
            {
                name = "Field structure collision without floor paint seams",
                indexFormat = _originalMesh.indexFormat,
                vertices = vertices,
                triangles = structureTriangles.ToArray()
            };
            _collisionMesh.RecalculateBounds();
            _collider.sharedMesh = _collisionMesh;
            if (hasCentralStep)
            {
                _centralStep = gameObject.AddComponent<BoxCollider>();
                _centralStep.center = stepBounds.center;
                _centralStep.size = stepBounds.size;
            }
        }

        private static bool TryGetCentralStepBounds(Vector3[] vertices, int[] triangles, Bounds field,
            out Bounds bounds)
        {
            bounds = default;
            bool found = false;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 a = vertices[triangles[i]];
                Vector3 b = vertices[triangles[i + 1]];
                Vector3 c = vertices[triangles[i + 2]];
                if (Mathf.Abs(a.y - field.max.y) > 0.0001f || Mathf.Abs(b.y - field.max.y) > 0.0001f ||
                    Mathf.Abs(c.y - field.max.y) > 0.0001f) continue;
                if (!found) { bounds = new Bounds(a, Vector3.zero); found = true; }
                bounds.Encapsulate(a);
                bounds.Encapsulate(b);
                bounds.Encapsulate(c);
            }
            if (!found || field.size.y < 0.05f) return false;
            bounds.SetMinMax(new Vector3(bounds.min.x, field.min.y, bounds.min.z), bounds.max);
            return true;
        }

        private static bool HasInwardTopFaces(Vector3[] vertices, int[] triangles, float top)
        {
            float normalY = 0f;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 a = vertices[triangles[i]];
                Vector3 b = vertices[triangles[i + 1]];
                Vector3 c = vertices[triangles[i + 2]];
                if (Mathf.Abs(a.y - top) > 0.0001f || Mathf.Abs(b.y - top) > 0.0001f ||
                    Mathf.Abs(c.y - top) > 0.0001f) continue;
                normalY += Vector3.Cross(b - a, c - a).y;
            }
            // Older BuildUnityMesh imports inverted the already-converted triangle
            // winding. Detect that data instead of flipping future correct meshes.
            return normalY < 0f;
        }

        public void AlignGround(BoxCollider ground)
        {
            if (_originalMesh == null || ground == null) return;
            Vector3 worldFloor = transform.TransformPoint(new Vector3(0, FloorHeight, 0));
            Vector3 localFloor = ground.transform.InverseTransformPoint(worldFloor);
            Vector3 center = ground.center;
            center.y = localFloor.y - ground.size.y * 0.5f;
            ground.center = center;
        }

        private void OnDestroy()
        {
            if (_collider != null && _collider.sharedMesh == _collisionMesh) _collider.sharedMesh = _originalMesh;
            if (_collisionMesh != null) Destroy(_collisionMesh);
            if (_centralStep != null) Destroy(_centralStep);
        }
    }
}
