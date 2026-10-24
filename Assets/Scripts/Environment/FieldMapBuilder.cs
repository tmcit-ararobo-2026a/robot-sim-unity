using System;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

namespace RobotSim.Environment
{
    [Serializable]
    public class IsaacMaterialMeta
    {
        public string name;
        public float[] color;
        public float metallic;
        public float smoothness;
    }

    [Serializable]
    public class IsaacSubmeshMeta
    {
        public int indexByteOffset;
        public int indexCount;
        public IsaacMaterialMeta material;
    }

    [Serializable]
    public class IsaacMeshEntry
    {
        public string name;
        public string path;
        public string category;
        public int vertexByteOffset;
        public int vertexCount;
        public IsaacSubmeshMeta[] submeshes;
    }

    [Serializable]
    public class IsaacSceneMeta
    {
        public float[] robotInitialPositionUnity;
        public float[] realSenseLocalPositionUnity;
        public float[] releasePointLocalPositionUnity;
        public float[] mid360LocalPositionUnity;
        public IsaacMeshEntry[] meshes;
    }

    [ExecuteInEditMode]
    public class FieldMapBuilder : MonoBehaviour
    {
        private static byte[] _cachedBinData;
        private static IsaacSceneMeta _cachedMeta;

        public static IsaacSceneMeta LoadSceneMeta(out byte[] binData)
        {
            string metaPath = Path.Combine(Application.streamingAssetsPath, "isaac_scene_meta.json");
            string binPath = Path.Combine(Application.streamingAssetsPath, "isaac_meshes.bin");

            if (_cachedMeta == null && File.Exists(metaPath))
            {
                _cachedMeta = JsonUtility.FromJson<IsaacSceneMeta>(File.ReadAllText(metaPath));
            }
            if (_cachedBinData == null && File.Exists(binPath))
            {
                _cachedBinData = File.ReadAllBytes(binPath);
            }

            binData = _cachedBinData;
            return _cachedMeta;
        }

        public static Material CreateIsaacMaterial(IsaacMaterialMeta m, bool isField = false)
        {
            Shader shader = Shader.Find("Standard");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Mobile/Diffuse");

            Color col = (m != null && m.color != null && m.color.Length >= 4)
                ? new Color(m.color[0], m.color[1], m.color[2], m.color[3])
                : new Color(0.8f, 0.8f, 0.8f, 1.0f);

            Material mat = new Material(shader)
            {
                name = m != null && !string.IsNullOrEmpty(m.name) ? m.name : "IsaacMat",
                color = col
            };

            float metallic = m != null ? m.metallic : 0.1f;
            float smoothness = m != null ? m.smoothness : 0.5f;
            if (isField)
            {
                metallic = 0.0f;
                smoothness = 0.15f;
            }
            if (mat.HasProperty("_Metallic")) mat.SetFloat("_Metallic", metallic);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);
            return mat;
        }

        public static Mesh BuildUnityMesh(IsaacMeshEntry entry, byte[] binData, out Material[] materials)
        {
            Mesh mesh = new Mesh
            {
                name = entry.name,
                indexFormat = IndexFormat.UInt32
            };

            int vCount = entry.vertexCount;
            float[] rawFloats = new float[vCount * 3];
            Buffer.BlockCopy(binData, entry.vertexByteOffset, rawFloats, 0, vCount * 12);

            Vector3[] vertices = new Vector3[vCount];
            for (int i = 0; i < vCount; i++)
            {
                int baseIdx = i * 3;
                vertices[i] = new Vector3(rawFloats[baseIdx], rawFloats[baseIdx + 1], rawFloats[baseIdx + 2]);
            }
            mesh.vertices = vertices;

            int subCount = entry.submeshes != null ? entry.submeshes.Length : 0;
            mesh.subMeshCount = subCount;
            materials = new Material[subCount];
            bool isField = entry.category == "field";

            for (int s = 0; s < subCount; s++)
            {
                IsaacSubmeshMeta sub = entry.submeshes[s];
                int idxCount = sub.indexCount;
                int[] indices = new int[idxCount];
                Buffer.BlockCopy(binData, sub.indexByteOffset, indices, 0, idxCount * 4);

                // Flip triangle winding order because USD (right-handed) -> Unity (left-handed) has det = -1
                for (int t = 0; t + 2 < idxCount; t += 3)
                {
                    int tmp = indices[t + 1];
                    indices[t + 1] = indices[t + 2];
                    indices[t + 2] = tmp;
                }

                mesh.SetTriangles(indices, s, false);
                materials[s] = CreateIsaacMaterial(sub.material, isField);
            }

            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        public void BuildField()
        {
            IsaacSceneMeta meta = LoadSceneMeta(out byte[] binData);
            if (meta == null || binData == null)
            {
                Debug.LogError("[FieldMapBuilder] Failed to load isaac_scene_meta.json or isaac_meshes.bin");
                return;
            }

            while (transform.childCount > 0)
            {
                DestroyImmediate(transform.GetChild(0).gameObject);
            }

            // Safety static ground collider at y = 0 matching Isaac Sim's groundPlane
            GameObject groundColliderObj = new GameObject("GroundPlaneCollider");
            groundColliderObj.transform.SetParent(transform, false);
            groundColliderObj.transform.localPosition = new Vector3(0f, -0.05f, 0f);
            BoxCollider groundBox = groundColliderObj.AddComponent<BoxCollider>();
            groundBox.size = new Vector3(30f, 0.1f, 30f);

            int fieldCount = 0;
            foreach (IsaacMeshEntry entry in meta.meshes)
            {
                if (entry.category != "field") continue;

                Mesh mesh = BuildUnityMesh(entry, binData, out Material[] mats);
                GameObject obj = new GameObject(entry.name);
                obj.transform.SetParent(transform, false);
                obj.transform.localPosition = Vector3.zero;
                obj.transform.localRotation = Quaternion.identity;

                MeshFilter mf = obj.AddComponent<MeshFilter>();
                mf.sharedMesh = mesh;

                MeshRenderer mr = obj.AddComponent<MeshRenderer>();
                mr.sharedMaterials = mats;

                MeshCollider mc = obj.AddComponent<MeshCollider>();
                mc.sharedMesh = mesh;

                fieldCount++;
            }

            Debug.Log($"[FieldMapBuilder] Constructed {fieldCount} exact Isaac Sim field meshes from main.usd!");
        }

        private void Start()
        {
            if (transform.childCount <= 1)
            {
                BuildField();
            }
        }
    }
}
