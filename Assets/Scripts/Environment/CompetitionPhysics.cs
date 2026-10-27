using System;
using UnityEngine;

namespace RobotSim.Environment
{
    // Approved dry-contact estimates, not measured material constants. Keep these
    // scoped to the field/opponent: the real robot's imported hierarchy is excluded.
    public static class CompetitionPhysics
    {
        public const float ClothMass = 0.048f;
        public const float BucketShellMass = 0.978f;
        public const string ResourceFolder = "Physics/SurfaceProfiles/";

        public readonly struct Surface
        {
            public readonly string Name;
            public readonly float StaticFriction;
            public readonly float DynamicFriction;
            public readonly float Bounciness;

            public Surface(string name, float staticFriction, float dynamicFriction, float bounciness)
            {
                Name = name;
                StaticFriction = staticFriction;
                DynamicFriction = dynamicFriction;
                Bounciness = bounciness;
            }
        }

        public static readonly Surface[] Surfaces =
        {
            new Surface("Floor", 0.60f, 0.45f, 0.05f),
            new Surface("Cloth", 0.75f, 0.55f, 0.00f),
            new Surface("Furniture", 0.45f, 0.30f, 0.05f),
            new Surface("Feet", 0.55f, 0.40f, 0.00f),
            new Surface("Bucket", 0.40f, 0.30f, 0.10f),
            new Surface("Wood", 0.50f, 0.35f, 0.05f),
            new Surface("FlagFabric", 0.60f, 0.45f, 0.00f),
            new Surface("FlagPole", 0.35f, 0.25f, 0.05f),
            new Surface("FlagBase", 0.55f, 0.40f, 0.00f)
        };

        public static PhysicsMaterial Material(string name)
        {
            var material = Resources.Load<PhysicsMaterial>(ResourceFolder + name);
            if (material == null)
                throw new InvalidOperationException("Missing competition contact material: " + name);
            return material;
        }

        public static void ApplyField(Transform field)
        {
            foreach (Transform prop in field)
            {
                bool chair = prop.name.StartsWith("chair", StringComparison.Ordinal);
                if (chair) SplitChairFeet(prop);
                var ground = prop.GetComponent<FieldGroundPhysics>();
                foreach (var collider in prop.GetComponentsInChildren<Collider>(true))
                {
                    if (!collider.enabled || collider.isTrigger) continue;
                    string surface;
                    if (ground != null)
                        surface = collider == ground.FloorCollider ? "Floor" : "Wood";
                    else if (prop.name == "GroundPlaneCollider") surface = "Floor";
                    else if (prop.name.StartsWith("bucket", StringComparison.Ordinal))
                        surface = prop.name.Contains("platform") ? "Wood" : "Bucket";
                    else if (prop.name.StartsWith("flag", StringComparison.Ordinal))
                        surface = collider.name == "Base" ? "FlagBase" :
                            collider.name == "Flag panel" ? "FlagFabric" : "FlagPole";
                    else if (chair || prop.name.StartsWith("desk", StringComparison.Ordinal))
                        surface = IsFoot(collider, chair) ? "Feet" : "Furniture";
                    else continue;
                    collider.sharedMaterial = Material(surface);
                }
            }
        }

        public static void ApplyOpponent(Transform opponent)
        {
            var chair = opponent.Find("Classroom_Chair_JIS6");
            if (chair != null) SplitChairFeet(chair);
            var bucket = opponent.Find("Opponent_Moving_Bucket");
            foreach (var collider in opponent.GetComponentsInChildren<Collider>(true))
            {
                if (!collider.enabled || collider.isTrigger) continue;
                bool onChair = chair != null && collider.transform.IsChildOf(chair);
                string surface = bucket != null && collider.transform.IsChildOf(bucket) ? "Bucket" :
                    onChair && IsFoot(collider, true) ? "Feet" : "Furniture";
                collider.sharedMaterial = Material(surface);
            }
            // The mounted bucket/chair share the kinematic opponent body. Adding
            // separate dynamic Rigidbodies would detach them from their mounts.
        }

        private static bool IsFoot(Collider collider, bool chair)
        {
            if (collider.name == "Foot") return true;
            return chair && collider.name == "Leg" && collider is BoxCollider box && box.size.y <= 0.025f;
        }

        private static void SplitChairFeet(Transform chair)
        {
            foreach (var leg in chair.GetComponentsInChildren<BoxCollider>(true))
            {
                if (!leg.enabled || leg.isTrigger || leg.name != "Leg" || leg.size.y <= 0.025f) continue;
                // The compact chair profile combines each cap and leg. Partition
                // the lowest 24 mm without changing their overall collision volume.
                var siblings = leg.GetComponents<BoxCollider>();
                if (Array.Exists(siblings, box => box != leg && box.size.y <= 0.025f)) continue;
                const float height = 0.024f;
                Vector3 center = leg.center;
                Vector3 size = leg.size;
                var foot = leg.gameObject.AddComponent<BoxCollider>();
                foot.center = center - Vector3.up * (size.y - height) * 0.5f;
                foot.size = new Vector3(size.x, height, size.z);
                foot.contactOffset = leg.contactOffset;
                foot.includeLayers = leg.includeLayers;
                foot.excludeLayers = leg.excludeLayers;
                foot.layerOverridePriority = leg.layerOverridePriority;
                leg.center = center + Vector3.up * height * 0.5f;
                leg.size = new Vector3(size.x, size.y - height, size.z);
            }
        }
    }
}
