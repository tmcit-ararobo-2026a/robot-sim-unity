using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using RobotSim.Control;
using RobotSim.Environment;

namespace RobotSim.Editor
{
    public static class ScenePhysicsBaker
    {
        [MenuItem("RobotSim/Bake and Save Scene Physics")]
        public static void BakeActiveScene() => BakeAndSave(SceneManager.GetActiveScene());

        public static void BakeAndSave(Scene scene)
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new InvalidOperationException("Stop Play mode before baking scene physics.");
            if (!scene.IsValid() || !scene.isLoaded || string.IsNullOrEmpty(scene.path))
                throw new InvalidOperationException("Save the scene before baking its physics.");

            string folder = "Assets/Generated/Physics/" + System.IO.Path.GetFileNameWithoutExtension(scene.path);
            EnsureFolder(folder);
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var builder in root.GetComponentsInChildren<FieldMapBuilder>(true))
                {
                    Mesh mesh = builder.BakePhysics();
                    if (mesh != null && !AssetDatabase.Contains(mesh))
                    {
                        string path = AssetDatabase.GenerateUniqueAssetPath(folder + "/FieldStructure.asset");
                        AssetDatabase.CreateAsset(mesh, path);
                    }
                    if (mesh != null) AssetDatabase.SaveAssetIfDirty(mesh);
                    EditorUtility.SetDirty(builder);
                    var ground = builder.GetComponentInChildren<FieldGroundPhysics>(true);
                    if (ground != null) EditorUtility.SetDirty(ground);
                }
                foreach (var drive in root.GetComponentsInChildren<KinematicOmniDrive>(true))
                {
                    drive.ConfigureBodyPhysics();
                    var body = drive.GetComponent<Rigidbody>();
                    string path = folder + "/OmniRollingContact.physicMaterial";
                    var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(path);
                    if (material == null)
                    {
                        material = new PhysicsMaterial("Omni drive rolling contact");
                        AssetDatabase.CreateAsset(material, path);
                    }
                    float friction = new SerializedObject(drive).FindProperty("rollingFriction").floatValue;
                    material.staticFriction = friction;
                    material.dynamicFriction = friction;
                    material.frictionCombine = PhysicsMaterialCombine.Minimum;
                    material.bounciness = 0f;
                    material.bounceCombine = PhysicsMaterialCombine.Minimum;
                    EditorUtility.SetDirty(material);
                    AssetDatabase.SaveAssetIfDirty(material);
                    foreach (var collider in drive.GetComponentsInChildren<Collider>(true))
                    {
                        if (collider.attachedRigidbody == body && !collider.isTrigger && collider.sharedMaterial == null)
                        {
                            collider.sharedMaterial = material;
                            EditorUtility.SetDirty(collider);
                        }
                    }
                    EditorUtility.SetDirty(body);
                }
            }
            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new InvalidOperationException("Could not save physics for " + scene.path);
            Debug.Log("[ScenePhysicsBaker] Saved edit-time Colliders and Rigidbodies: " + scene.path);
        }

        private static void EnsureFolder(string path)
        {
            string parent = System.IO.Path.GetDirectoryName(path).Replace('\\', '/');
            if (!AssetDatabase.IsValidFolder(parent)) EnsureFolder(parent);
            if (!AssetDatabase.IsValidFolder(path))
                AssetDatabase.CreateFolder(parent, System.IO.Path.GetFileName(path));
        }
    }
}
