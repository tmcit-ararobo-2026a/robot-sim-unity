using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RobotSim.Environment
{
    // Primitive compound colliders keep movable CAD props legal for dynamic PhysX bodies.
    public static class FieldPropPhysics
    {
        public static bool TryGetMass(string name, float deskMass, float chairMass, float bucketMass,
            float flagMass, float platformMass, out float mass)
        {
            mass = 0f;
            if (name.StartsWith("desk", StringComparison.Ordinal)) mass = deskMass;
            else if (name.StartsWith("chair", StringComparison.Ordinal)) mass = chairMass;
            else if (name.StartsWith("bucket", StringComparison.Ordinal))
                mass = name.Contains("platform") ? platformMass : bucketMass;
            else if (name.StartsWith("flag", StringComparison.Ordinal)) mass = flagMass;
            return mass > 0f;
        }

        public static Rigidbody MakeMovable(GameObject geometry, float mass)
        {
            var existing = geometry.GetComponent<Rigidbody>();
            if (existing != null) return existing;
            var filter = geometry.GetComponent<MeshFilter>();
            if (filter == null || filter.sharedMesh == null) return null;
            Mesh mesh = filter.sharedMesh;
            Bounds bounds = mesh.bounds;
            string propName = geometry.name;

            // Imported vertices are in field coordinates. Put the body origin at the prop
            // rather than at the field origin, preserving the rendered mesh's world pose.
            var root = new GameObject(propName);
            SceneManager.MoveGameObjectToScene(root, geometry.scene);
            root.transform.SetParent(geometry.transform.parent, false);
            root.transform.SetPositionAndRotation(geometry.transform.TransformPoint(bounds.center), geometry.transform.rotation);
            root.transform.localScale = geometry.transform.localScale;
            geometry.transform.SetParent(root.transform, true);
            geometry.name = "Geometry";
            geometry.isStatic = false;
            foreach (var collider in geometry.GetComponents<Collider>())
            {
                collider.enabled = false;
                if (Application.isPlaying) UnityEngine.Object.Destroy(collider);
                else UnityEngine.Object.DestroyImmediate(collider);
            }

            if (!CompactBoxColliders.TryBuild(root, mesh, true))
            {
                if (propName.Contains("platform")) Box(root, new Bounds(Vector3.zero, bounds.size));
                else if (propName.StartsWith("bucket", StringComparison.Ordinal)) BuildBucket(root.transform, bounds.size);
                else if (propName.StartsWith("flag", StringComparison.Ordinal)) BuildFlag(root.transform, mesh);
                else BuildCadParts(root, mesh);
            }

            var body = root.AddComponent<Rigidbody>();
            body.mass = mass;
            body.useGravity = true;
            body.isKinematic = false;
            body.linearDamping = 0.05f;
            body.angularDamping = 0.1f;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            return body;
        }

        public static void ConfigureStaticPlatform(GameObject platform)
        {
            var filter = platform.GetComponent<MeshFilter>();
            if (filter == null || filter.sharedMesh == null) return;
            // These CAD platforms are solid cuboids. A box has a closed top even
            // when an older imported mesh has its triangle winding reversed.
            foreach (var meshCollider in platform.GetComponents<MeshCollider>()) meshCollider.enabled = false;
            if (platform.GetComponent<BoxCollider>() == null) Box(platform, filter.sharedMesh.bounds);
        }

        private static void Box(GameObject obj, Bounds bounds)
        {
            var collider = obj.AddComponent<BoxCollider>();
            collider.center = bounds.center;
            Vector3 size = bounds.size;
            collider.size = new Vector3(Mathf.Max(size.x, 0.004f), Mathf.Max(size.y, 0.004f), Mathf.Max(size.z, 0.004f));
        }

        private static void BuildBucket(Transform root, Vector3 size)
        {
            // Preserve the open top; a single convex hull would seal the bucket.
            const int sides = 24;
            const float thickness = 0.005f;
            float radius = Mathf.Min(size.x, size.z) * 0.5f - thickness * 0.5f;
            float width = 2f * radius * Mathf.Tan(Mathf.PI / sides) + thickness;
            Box(root.gameObject, new Bounds(new Vector3(0, -size.y * 0.5f + thickness * 0.5f, 0),
                new Vector3(size.x * 0.72f, thickness, size.z * 0.72f)));
            for (int i = 0; i < sides; i++)
            {
                float angle = i * 2f * Mathf.PI / sides;
                var wall = new GameObject("Bucket wall " + i);
                wall.transform.SetParent(root, false);
                wall.transform.localPosition = new Vector3(Mathf.Sin(angle) * radius, 0, Mathf.Cos(angle) * radius);
                wall.transform.localRotation = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
                Box(wall, new Bounds(Vector3.zero, new Vector3(width, size.y, thickness)));
            }
        }

        private static void BuildCadParts(GameObject root, Mesh mesh)
        {
            Vector3[] vertices = mesh.vertices;
            int[] parent = new int[vertices.Length];
            var welded = new Dictionary<Vector3Int, int>();
            for (int i = 0; i < vertices.Length; i++)
            {
                parent[i] = i;
                Vector3 v = vertices[i] * 100000f;
                var key = new Vector3Int(Mathf.RoundToInt(v.x), Mathf.RoundToInt(v.y), Mathf.RoundToInt(v.z));
                if (welded.TryGetValue(key, out int previous)) parent[i] = previous;
                else welded.Add(key, i);
            }
            int[] triangles = mesh.triangles;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = Find(parent, triangles[i]);
                parent[Find(parent, triangles[i + 1])] = a;
                parent[Find(parent, triangles[i + 2])] = a;
            }
            var parts = new Dictionary<int, Bounds>();
            for (int i = 0; i < vertices.Length; i++)
            {
                int part = Find(parent, i);
                Vector3 v = vertices[i] - mesh.bounds.center;
                if (!parts.TryGetValue(part, out Bounds bounds)) bounds = new Bounds(v, Vector3.zero);
                bounds.Encapsulate(v);
                parts[part] = bounds;
            }
            foreach (Bounds part in parts.Values)
            {
                // Screws and tiny trim stay visible without adding solver contacts.
                if (part.size.magnitude < 0.025f) continue;
                Box(root, part);
            }
            if (root.GetComponent<Collider>() == null) Box(root, new Bounds(Vector3.zero, mesh.bounds.size));
        }

        private static int Find(int[] parents, int i)
        {
            while (parents[i] != i)
            {
                parents[i] = parents[parents[i]];
                i = parents[i];
            }
            return i;
        }

        private static void BuildFlag(Transform root, Mesh mesh)
        {
            // Horizontal slices separate the wide base/flag from the narrow pole.
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            Bounds meshBounds = mesh.bounds;
            const int slices = 30;
            for (int slice = 0; slice < slices; slice++)
            {
                float low = meshBounds.min.y + meshBounds.size.y * slice / slices;
                float high = meshBounds.min.y + meshBounds.size.y * (slice + 1) / slices;
                bool found = false;
                Bounds part = default;
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    for (int edge = 0; edge < 3; edge++)
                    {
                        Vector3 a = vertices[triangles[i + edge]];
                        Vector3 b = vertices[triangles[i + (edge + 1) % 3]];
                        if (a.y >= low && a.y <= high) Include(ref part, ref found, a - meshBounds.center);
                        if (Mathf.Abs(b.y - a.y) < 1e-7f) continue;
                        float t = (low - a.y) / (b.y - a.y);
                        if (t >= 0 && t <= 1) Include(ref part, ref found, Vector3.Lerp(a, b, t) - meshBounds.center);
                        t = (high - a.y) / (b.y - a.y);
                        if (t >= 0 && t <= 1) Include(ref part, ref found, Vector3.Lerp(a, b, t) - meshBounds.center);
                    }
                }
                if (found) Box(root.gameObject, part);
            }
        }

        private static void Include(ref Bounds bounds, ref bool found, Vector3 point)
        {
            if (!found) { bounds = new Bounds(point, Vector3.zero); found = true; }
            else bounds.Encapsulate(point);
        }
    }
}
