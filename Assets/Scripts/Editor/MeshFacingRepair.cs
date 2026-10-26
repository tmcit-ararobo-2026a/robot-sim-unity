using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using RobotSim.Environment;

namespace RobotSim.Editor
{
    public static class MeshFacingRepair
    {
        [MenuItem("RobotSim/Repair Imported Mesh Facing and Save")]
        public static void RepairActiveScene()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new InvalidOperationException("Repair imported meshes in Edit mode.");
            var scene = SceneManager.GetActiveScene();
            if (string.IsNullOrEmpty(scene.path))
                throw new InvalidOperationException("Save the scene before repairing its meshes.");

            var meta = FieldMapBuilder.LoadSceneMeta(out _);
            if (meta == null) throw new InvalidOperationException("Scene mesh metadata is unavailable.");
            var importedNames = new HashSet<string>();
            foreach (var entry in meta.meshes) importedNames.Add(entry.name);
            var meshes = new HashSet<Mesh>();
            foreach (var root in scene.GetRootGameObjects())
                foreach (var filter in root.GetComponentsInChildren<MeshFilter>(true))
                    if (filter.sharedMesh != null && (importedNames.Contains(filter.sharedMesh.name) || IsOpponentCad(filter.sharedMesh.name)))
                        meshes.Add(filter.sharedMesh);

            // Builders reuse these cached CAD meshes, including the other team colour.
            foreach (string guid in AssetDatabase.FindAssets("t:Mesh", new[] { "Assets/Resources/Opponent" }))
            {
                var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(AssetDatabase.GUIDToAssetPath(guid));
                if (mesh != null && IsOpponentCad(mesh.name)) meshes.Add(mesh);
            }

            int repaired = 0;
            foreach (var mesh in meshes)
            {
                if (!RepairIfInward(mesh)) continue;
                repaired++;
                if (AssetDatabase.Contains(mesh)) AssetDatabase.SaveAssetIfDirty(mesh);
            }
            if (repaired > 0)
            {
                EditorSceneManager.MarkSceneDirty(scene);
                if (!EditorSceneManager.SaveScene(scene)) throw new InvalidOperationException("Could not save the repaired scene.");
            }
            Debug.Log("[MeshFacingRepair] Repaired " + repaired + " inward-facing imported meshes.");
        }

        private static bool IsOpponentCad(string name) => name == "JIS6_Chair_CAD" || name.StartsWith("PO24A_Bucket_CAD", StringComparison.Ordinal);

        public static bool RepairIfInward(Mesh mesh)
        {
            if (SignedVolume(mesh) >= -1e-9) return false;
            Undo.RecordObject(mesh, "Repair imported mesh facing");
            for (int submesh = 0; submesh < mesh.subMeshCount; submesh++)
            {
                int[] indices = mesh.GetTriangles(submesh);
                for (int i = 0; i < indices.Length; i += 3)
                {
                    int old = indices[i + 1];
                    indices[i + 1] = indices[i + 2];
                    indices[i + 2] = old;
                }
                mesh.SetTriangles(indices, submesh, false);
            }
            mesh.RecalculateNormals();
            EditorUtility.SetDirty(mesh);
            return true;
        }

        public static double SignedVolume(Mesh mesh)
        {
            Vector3[] vertices = mesh.vertices;
            int[] indices = mesh.triangles;
            Vector3 origin = mesh.bounds.center;
            double volume = 0;
            for (int i = 0; i < indices.Length; i += 3)
            {
                Vector3 a = vertices[indices[i]] - origin;
                Vector3 b = vertices[indices[i + 1]] - origin;
                Vector3 c = vertices[indices[i + 2]] - origin;
                volume += Vector3.Dot(a, Vector3.Cross(b, c)) / 6.0;
            }
            return volume;
        }
    }
}
