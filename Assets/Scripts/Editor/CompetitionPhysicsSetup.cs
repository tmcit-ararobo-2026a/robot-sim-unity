using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using RobotSim.Environment;
using RobotSim.Opponent;

namespace RobotSim.Editor
{
    public static class CompetitionPhysicsSetup
    {
        [MenuItem("RobotSim/Apply Approved Field Physics and Save")]
        public static void ApplyActiveScene() => ApplyAndSave(SceneManager.GetActiveScene());

        public static void EnsureMaterials()
        {
            string folder = "Assets/Resources/" + CompetitionPhysics.ResourceFolder.TrimEnd('/');
            EnsureFolder(folder);
            foreach (var surface in CompetitionPhysics.Surfaces)
            {
                string path = folder + "/" + surface.Name + ".physicMaterial";
                var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(path);
                if (material == null)
                {
                    material = new PhysicsMaterial(surface.Name);
                    AssetDatabase.CreateAsset(material, path);
                }
                material.staticFriction = surface.StaticFriction;
                material.dynamicFriction = surface.DynamicFriction;
                material.bounciness = surface.Bounciness;
                material.frictionCombine = PhysicsMaterialCombine.Average;
                material.bounceCombine = PhysicsMaterialCombine.Minimum;
                EditorUtility.SetDirty(material);
                AssetDatabase.SaveAssetIfDirty(material);
            }
        }

        public static void ApplyAndSave(Scene scene)
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new InvalidOperationException("Stop Play mode before applying field physics.");
            if (!scene.IsValid() || !scene.isLoaded || string.IsNullOrEmpty(scene.path))
                throw new InvalidOperationException("A saved scene is required.");
            EnsureMaterials();
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var builder in root.GetComponentsInChildren<FieldMapBuilder>(true))
                {
                    var settings = new SerializedObject(builder);
                    settings.FindProperty("bucketMass").floatValue = CompetitionPhysics.BucketShellMass;
                    settings.ApplyModifiedPropertiesWithoutUndo();
                    Mesh mesh = builder.BakePhysics();
                    if (mesh != null && !AssetDatabase.Contains(mesh))
                    {
                        string folder = "Assets/Generated/Physics/" + System.IO.Path.GetFileNameWithoutExtension(scene.path);
                        EnsureFolder(folder);
                        AssetDatabase.CreateAsset(mesh, AssetDatabase.GenerateUniqueAssetPath(folder + "/FieldStructure.asset"));
                    }
                }
                foreach (var opponent in root.GetComponentsInChildren<OpponentRobotController>(true))
                    CompetitionPhysics.ApplyOpponent(opponent.transform);
            }
            // Deliberately do not call ScenePhysicsBaker: its robot pass assigns
            // rolling-contact materials to the real robot, outside this operation.
            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new InvalidOperationException("Could not save " + scene.path);
        }

        public static void ApplyClothPrefab()
        {
            const string path = "Assets/Prefab/cloth.prefab";
            EnsureMaterials();
            var cloth = PrefabUtility.LoadPrefabContents(path);
            try
            {
                var body = cloth.GetComponent<Rigidbody>();
                if (body == null) throw new InvalidOperationException("Cloth Rigidbody is missing.");
                body.mass = CompetitionPhysics.ClothMass;
                foreach (var collider in cloth.GetComponentsInChildren<Collider>(true))
                    if (!collider.isTrigger) collider.sharedMaterial = CompetitionPhysics.Material("Cloth");
                PrefabUtility.SaveAsPrefabAsset(cloth, path);
            }
            finally { PrefabUtility.UnloadPrefabContents(cloth); }
        }

        private static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path)) return;
            string parent = System.IO.Path.GetDirectoryName(path).Replace('\\', '/');
            EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, System.IO.Path.GetFileName(path));
        }
    }
}
