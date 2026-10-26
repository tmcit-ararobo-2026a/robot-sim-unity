using System;
using UnityEngine;

namespace RobotSim.Environment
{
    // Mesh-local, centered layouts keep the same fit when an object is moved/rotated.
    public static class CompactBoxColliders
    {
        [Serializable] public sealed class Box
        {
            public string label;
            public Vector3 center;
            public Vector3 size;
            public Quaternion rotation;
        }

        [Serializable] public sealed class Profile
        {
            public string meshName;
            public int vertexCount;
            public Vector3 boundsSize;
            public Box[] boxes;
        }

        [Serializable] private sealed class ProfileSet { public Profile[] profiles; }
        private static ProfileSet _profiles;
        public const string ContainerName = "CollisionBoxes";

        public static void ReloadProfiles() => _profiles = null;

        public static Profile Find(Mesh mesh)
        {
            if (mesh == null) return null;
            if (_profiles == null)
            {
                var asset = Resources.Load<TextAsset>("Physics/CompactBoxProfiles");
                if (asset == null) return null;
                _profiles = JsonUtility.FromJson<ProfileSet>(asset.text);
            }
            foreach (var profile in _profiles.profiles)
                if (profile.meshName == mesh.name && profile.vertexCount == mesh.vertexCount &&
                    Vector3.Distance(profile.boundsSize, mesh.bounds.size) < 0.0001f) return profile;
            return null;
        }

        public static bool TryBuild(GameObject owner, Mesh mesh, bool centered = false, PhysicsMaterial material = null)
        {
            var profile = Find(mesh);
            if (profile == null) return false;
            var container = new GameObject(ContainerName);
            container.transform.SetParent(owner.transform, false);
            container.transform.localPosition = centered ? Vector3.zero : mesh.bounds.center;
            foreach (var box in profile.boxes)
            {
                var part = new GameObject(box.label);
                part.transform.SetParent(container.transform, false);
                part.transform.localPosition = box.center;
                part.transform.localRotation = box.rotation;
                var collider = part.AddComponent<BoxCollider>();
                collider.size = box.size;
                collider.sharedMaterial = material;
            }
            return true;
        }
    }
}
