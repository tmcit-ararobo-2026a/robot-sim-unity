using System.Collections.Generic;
using UnityEngine;

namespace RobotSim.Environment
{
    // CAD paint/tile strips are 1 mm solids. Their vertical edges must not catch
    // sliding furniture or the chassis; collision uses one seamless floor instead.
    [RequireComponent(typeof(MeshFilter))]
    public class FieldGroundPhysics : MonoBehaviour
    {
        [SerializeField, HideInInspector] private bool collisionBaked;
        [SerializeField, HideInInspector] private BoxCollider floorCollider;
        private Mesh _originalMesh;
        private Mesh _collisionMesh;
        private MeshCollider _collider;
        private BoxCollider _centralStep;
        public float FloorHeight { get; private set; }
        public BoxCollider FloorCollider => floorCollider;

        public BoxCollider EnsureFloorCollider(BoxCollider legacyGround = null)
        {
            if (floorCollider != null) return floorCollider;
            floorCollider = gameObject.AddComponent<BoxCollider>();
            if (legacyGround != null)
            {
                floorCollider.center = transform.InverseTransformPoint(legacyGround.transform.TransformPoint(legacyGround.center));
                Vector3 scale = legacyGround.transform.lossyScale;
                Vector3 targetScale = transform.lossyScale;
                floorCollider.size = Vector3.Scale(legacyGround.size,
                    new Vector3(Mathf.Abs(scale.x / targetScale.x), Mathf.Abs(scale.y / targetScale.y),
                        Mathf.Abs(scale.z / targetScale.z)));
                floorCollider.sharedMaterial = legacyGround.sharedMaterial;
                floorCollider.isTrigger = legacyGround.isTrigger;
                floorCollider.enabled = legacyGround.enabled;
                floorCollider.contactOffset = legacyGround.contactOffset;
                floorCollider.includeLayers = legacyGround.includeLayers;
                floorCollider.excludeLayers = legacyGround.excludeLayers;
                floorCollider.layerOverridePriority = legacyGround.layerOverridePriority;
            }
            else
            {
                floorCollider.size = new Vector3(1f, 0.1f, 1f);
                FitFloorFootprint(floorCollider);
                AlignGround(floorCollider);
            }
            return floorCollider;
        }

        public void FitFloorToField() => FitFloorFootprint(EnsureFloorCollider());

        private void FitFloorFootprint(BoxCollider ground)
        {
            var mesh = GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null) return;
            Bounds bounds = mesh.bounds;
            Vector3 center = ground.center;
            center.x = bounds.center.x;
            center.z = bounds.center.z;
            ground.center = center;
            Vector3 size = ground.size;
            size.x = bounds.size.x;
            size.z = bounds.size.z;
            ground.size = size;
        }

        private void Awake()
        {
            if (!collisionBaked) ConfigureCollisionGeometry();
        }

        // The Editor persists the returned mesh as an asset before saving the scene.
        public Mesh BakeCollisionGeometry(BoxCollider ground)
        {
            if (Application.isPlaying)
                throw new System.InvalidOperationException("Bake field collision in Edit mode.");
            if (collisionBaked)
            {
                var collider = GetComponent<MeshCollider>();
                return collider != null ? collider.sharedMesh : null;
            }
            ConfigureCollisionGeometry();
            AlignGround(ground);
            if (_collisionMesh == null && transform.Find(CompactBoxColliders.ContainerName) == null)
                throw new System.InvalidOperationException("Field collision requires a source mesh.");
            Mesh bakedMesh = _collisionMesh;
            collisionBaked = true;
            // Baked geometry belongs to the saved scene/assets, not runtime cleanup.
            _originalMesh = null;
            _collisionMesh = null;
            _centralStep = null;
            return bakedMesh;
        }

        private void ConfigureCollisionGeometry()
        {
            _collider = GetComponent<MeshCollider>();
            _originalMesh = GetComponent<MeshFilter>().sharedMesh;
            if (_originalMesh == null && _collider != null) _originalMesh = _collider.sharedMesh;
            if (_originalMesh == null) return;
            Vector3[] vertices = _originalMesh.vertices;
            FloorHeight = _originalMesh.bounds.min.y + 0.001f;
            if (CompactBoxColliders.TryBuild(gameObject, _originalMesh))
            {
                if (_collider != null) _collider.enabled = false;
                return;
            }
            if (_collider == null) _collider = gameObject.AddComponent<MeshCollider>();
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
            var mesh = _originalMesh != null ? _originalMesh : GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null || ground == null) return;
            FloorHeight = mesh.bounds.min.y + 0.001f;
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
