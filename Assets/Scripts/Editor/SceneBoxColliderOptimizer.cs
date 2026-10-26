using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using RobotSim.Control;
using RobotSim.Environment;
using RobotSim.Opponent;

namespace RobotSim.Editor
{
    public static class SceneBoxColliderOptimizer
    {
        [MenuItem("RobotSim/Fit Compact Box Colliders and Save")]
        public static void OptimizeActiveScene() => OptimizeAndSave(SceneManager.GetActiveScene());

        public static void OptimizeAndSave(Scene scene)
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new InvalidOperationException("Stop Play mode before fitting colliders.");
            CompactBoxColliders.ReloadProfiles();
            // Also handles scenes whose field props have not been baked yet.
            ScenePhysicsBaker.BakeAndSave(scene);
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var builder in root.GetComponentsInChildren<FieldMapBuilder>(true))
                {
                    var sources = builder.GetComponentsInChildren<MeshFilter>(true);
                    foreach (var source in sources)
                    {
                        if (CompactBoxColliders.Find(source.sharedMesh) == null) continue;
                        Transform owner = source.GetComponentInParent<Rigidbody>()?.transform ?? source.transform;
                        Replace(owner, source);
                    }
                }
                foreach (var drive in root.GetComponentsInChildren<KinematicOmniDrive>(true))
                {
                    MeshFilter source = null;
                    foreach (var candidate in drive.GetComponentsInChildren<MeshFilter>(true))
                        if (candidate.sharedMesh != null && CompactBoxColliders.Find(candidate.sharedMesh) != null)
                            source = candidate;
                    if (source == null) throw new InvalidOperationException("No CAD collision layout for " + drive.name);
                    PhysicsMaterial material = null;
                    foreach (var collider in drive.GetComponentsInChildren<Collider>(true))
                        if (collider.sharedMaterial != null) { material = collider.sharedMaterial; break; }
                    Replace(drive.transform, source, material);
                }
                foreach (var opponent in root.GetComponentsInChildren<OpponentRobotController>(true))
                    FitOpponent(opponent.transform);
            }
            EditorSceneManager.MarkSceneDirty(scene);
            ScenePhysicsBaker.BakeAndSave(scene);
        }

        private static void ClearSolids(Transform owner)
        {
            var groundPhysics = owner.GetComponent<FieldGroundPhysics>();
            var floor = groundPhysics != null ? groundPhysics.FloorCollider : null;
            foreach (var collider in owner.GetComponentsInChildren<Collider>(true))
                if (!collider.isTrigger && collider != floor) UnityEngine.Object.DestroyImmediate(collider);
            // Remove only generated collision children; preserve every visual transform.
            var remove = new List<GameObject>();
            foreach (var child in owner.GetComponentsInChildren<Transform>(true))
                if (child != owner && (child.name == CompactBoxColliders.ContainerName ||
                    child.name.StartsWith("Bucket wall ", StringComparison.Ordinal) ||
                    child.name.StartsWith("Bucket_WallCollider_", StringComparison.Ordinal)))
                    remove.Add(child.gameObject);
            foreach (var child in remove) if (child != null) UnityEngine.Object.DestroyImmediate(child);
        }

        private static void Replace(Transform owner, MeshFilter source, PhysicsMaterial material = null)
        {
            ClearSolids(owner);
            if (!CompactBoxColliders.TryBuild(source.gameObject, source.sharedMesh, false, material))
                throw new InvalidOperationException("No matching collision layout for " + source.name);
        }

        private static Bounds MeshBounds(Transform owner, params Transform[] objects)
        {
            Bounds bounds = default;
            bool found = false;
            foreach (var item in objects)
            {
                if (item == null) continue;
                foreach (var filter in item.GetComponentsInChildren<MeshFilter>(true))
                {
                    if (filter.sharedMesh == null) continue;
                    Bounds mesh = filter.sharedMesh.bounds;
                    Matrix4x4 matrix = owner.worldToLocalMatrix * filter.transform.localToWorldMatrix;
                    for (int corner = 0; corner < 8; corner++)
                    {
                        Vector3 point = mesh.center + Vector3.Scale(mesh.extents,
                            new Vector3((corner & 1) == 0 ? -1 : 1, (corner & 2) == 0 ? -1 : 1,
                                (corner & 4) == 0 ? -1 : 1));
                        point = matrix.MultiplyPoint3x4(point);
                        if (!found) { bounds = new Bounds(point, Vector3.zero); found = true; }
                        else bounds.Encapsulate(point);
                    }
                }
            }
            if (!found) throw new InvalidOperationException("Missing visual mesh under " + owner.name);
            return bounds;
        }

        private static void Box(Transform owner, Bounds bounds)
        {
            var collider = owner.gameObject.AddComponent<BoxCollider>();
            collider.center = bounds.center;
            collider.size = bounds.size;
        }

        private static void FitOpponent(Transform root)
        {
            ClearSolids(root);
            var chassis = root.Find("Chassis_OmniBase");
            Box(chassis, MeshBounds(chassis, chassis.Find("Bottom_Deck_Plate"), chassis.Find("Upper_Deck_Plate"),
                chassis.Find("Armor_Front"), chassis.Find("Armor_Back"), chassis.Find("Armor_Left"), chassis.Find("Armor_Right")));
            for (int i = 0; i < 4; i++)
            {
                var wheel = chassis.Find("OmniWheel_" + i);
                Box(wheel, MeshBounds(wheel, wheel));
            }
            var launcher = root.Find("Cloth_Launcher_Mechanism");
            foreach (string name in new[] { "Shooter_Side_L", "Shooter_Side_R" })
            {
                var side = launcher.Find(name);
                Box(side, MeshBounds(side, side));
            }
            Box(launcher, MeshBounds(launcher, launcher.Find("Shooter_Bed"), launcher.Find("Lower_Drive_Roller"),
                launcher.Find("Upper_Drive_Roller")));
            var chair = root.Find("Classroom_Chair_JIS6").GetComponent<MeshFilter>();
            if (!CompactBoxColliders.TryBuild(chair.gameObject, chair.sharedMesh))
                throw new InvalidOperationException("Missing opponent chair collision layout.");
            var mascot = root.Find("Team_Mascot");
            Box(mascot, MeshBounds(mascot, mascot.Find("Mascot_Torso_Bib"), mascot.Find("Safety_SeatBelt_Waist")));
            var head = mascot.Find("Mascot_Head");
            Box(head, MeshBounds(head, head, mascot.Find("Mascot_Helmet"), mascot.Find("Mascot_Helmet_Visor")));
            for (int i = 0; i < 2; i++)
            {
                var arm = mascot.Find("Mascot_Arm_" + i);
                Box(arm, MeshBounds(arm, arm, mascot.Find("Mascot_Hand_" + i)));
                var thigh = mascot.Find("Mascot_Thigh_" + i);
                Box(thigh, MeshBounds(thigh, thigh));
                var shin = mascot.Find("Mascot_Shin_" + i);
                Box(shin, MeshBounds(shin, shin, mascot.Find("Mascot_Shoe_" + i)));
            }
            var tower = root.Find("Bucket_Support_Tower");
            foreach (string name in new[] { "Tower_Pillar_L", "Tower_Pillar_R", "Tower_CrossBrace_Low", "Tower_CrossBrace_High", "Bucket_Base_MountPlate" })
            {
                var piece = tower.Find(name);
                Box(piece, MeshBounds(piece, piece));
            }
            var bucket = root.Find("Opponent_Moving_Bucket").GetComponent<MeshFilter>();
            if (!CompactBoxColliders.TryBuild(bucket.gameObject, bucket.sharedMesh))
                throw new InvalidOperationException("Missing opponent bucket collision layout.");
        }
    }
}
