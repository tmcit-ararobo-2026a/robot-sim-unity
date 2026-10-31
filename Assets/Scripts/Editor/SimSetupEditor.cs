#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using ROS2;
using RobotSim.Environment;
using RobotSim.Control;
using RobotSim.Sensors;
using RobotSim.Shooter;
using RobotSim.UI;

namespace RobotSim.Editor
{
    public static class SimSetupEditor
    {
        private static readonly Vector3 SceneAlignOffset = new Vector3(0.024f, -0.046f, -0.300f);
        private static readonly Quaternion SceneAlignRotation = Quaternion.Euler(0f, -90f, 0f);

        private static Vector3 AlignPointToMap(Vector3 rawUnityPos)
        {
            return (SceneAlignRotation * rawUnityPos) + SceneAlignOffset;
        }

        private static Material CreateMat(string name, Color color, float metallic = 0.5f, float smoothness = 0.5f)
        {
            Shader shader = Shader.Find("Standard");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Mobile/Diffuse");
            Material mat = new Material(shader) { name = name, color = color };
            if (mat.HasProperty("_Metallic")) mat.SetFloat("_Metallic", metallic);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);
            return mat;
        }

        [MenuItem("RobotSim/Build Complete Simulation Scene")]
        public static void BuildScene()
        {
            var clothPrefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefab/cloth.prefab");
            if (clothPrefab == null)
                throw new System.InvalidOperationException("Assets/Prefab/cloth.prefab is required to build the scene.");

            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // 1. Isaac Sim Environment Lighting (4 DistantLights from /Enviroment/DistantLight..03)
            GameObject envObj = new GameObject("Enviroment");
            Vector3[] lightEulers = new Vector3[]
            {
                new Vector3(50f, -125f, 0f),
                new Vector3(45f, -35f, 0f),
                new Vector3(45f, 55f, 0f),
                new Vector3(45f, 145f, 0f)
            };
            float[] lightIntensities = new float[] { 0.55f, 0.22f, 0.18f, 0.18f };
            for (int i = 0; i < lightEulers.Length; i++)
            {
                GameObject lObj = new GameObject(i == 0 ? "DistantLight" : $"DistantLight_0{i}");
                lObj.transform.SetParent(envObj.transform, false);
                lObj.transform.rotation = Quaternion.Euler(lightEulers[i]);
                Light l = lObj.AddComponent<Light>();
                l.type = LightType.Directional;
                l.intensity = lightIntensities[i];
                l.color = new Color(1.0f, 0.99f, 0.97f);
                if (i == 0) l.shadows = LightShadows.Soft;
            }
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.32f, 0.34f, 0.36f);

            // 2. Spectator Camera (framed overlooking robot & field)
            GameObject camObj = new GameObject("Spectator Camera");
            Camera specCam = camObj.AddComponent<Camera>();
            specCam.backgroundColor = new Color(0.22f, 0.24f, 0.27f);
            specCam.clearFlags = CameraClearFlags.SolidColor;
            specCam.depth = 1;
            camObj.tag = "MainCamera";
            camObj.transform.position = AlignPointToMap(new Vector3(-5.8f, 3.2f, -4.5f));
            camObj.transform.LookAt(AlignPointToMap(new Vector3(-1.5f, 0.5f, 0.0f)));

            // 3. ROS2 Unity Manager
            GameObject ros2Obj = new GameObject("ROS2Manager");
            ros2Obj.AddComponent<ROS2UnityComponent>();

            // 4. Load Isaac Sim Scene Metadata & Binary Meshes
            IsaacSceneMeta meta = FieldMapBuilder.LoadSceneMeta(out byte[] binData);

            // 5. Build Exact Isaac Sim Field (/World/field/*) aligned to nhk2026_map.json
            GameObject fieldObj = new GameObject("Field");
            fieldObj.transform.position = SceneAlignOffset;
            fieldObj.transform.rotation = SceneAlignRotation;
            FieldMapBuilder mapBuilder = fieldObj.AddComponent<FieldMapBuilder>();
            mapBuilder.BuildField();

            // 6. Build Exact Isaac Sim Robot (/World/base_link) aligned to nhk2026_map.json
            GameObject robotObj = new GameObject("Robot");
            Vector3 rawRobotPos = (meta != null && meta.robotInitialPositionUnity != null && meta.robotInitialPositionUnity.Length == 3)
                ? new Vector3(meta.robotInitialPositionUnity[0], meta.robotInitialPositionUnity[1], meta.robotInitialPositionUnity[2])
                : new Vector3(-3.841952f, 0.021873f, -2.275948f);
            robotObj.transform.position = AlignPointToMap(rawRobotPos);
            robotObj.transform.rotation = SceneAlignRotation;

            Rigidbody rb = robotObj.AddComponent<Rigidbody>();
            rb.mass = 25.0f;
            rb.linearDamping = 0.5f;
            rb.angularDamping = 1.0f;
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

            BoxCollider col = robotObj.AddComponent<BoxCollider>();
            col.size = new Vector3(0.75f, 1.15f, 0.85f);
            col.center = new Vector3(0f, 0.58f, 0f);

            // Attach the 471,367-vertex CAD mesh from v3_3.usd / main.usd
            if (meta != null && binData != null && meta.meshes != null)
            {
                foreach (var entry in meta.meshes)
                {
                    if (entry.category == "robot")
                    {
                        Mesh cadMesh = FieldMapBuilder.BuildUnityMesh(entry, binData, out Material[] cadMats);
                        GameObject cadVisualObj = new GameObject("base_link");
                        cadVisualObj.transform.SetParent(robotObj.transform, false);
                        cadVisualObj.transform.localPosition = Vector3.zero;
                        cadVisualObj.transform.localRotation = Quaternion.identity;

                        MeshFilter mf = cadVisualObj.AddComponent<MeshFilter>();
                        mf.sharedMesh = cadMesh;

                        MeshRenderer mr = cadVisualObj.AddComponent<MeshRenderer>();
                        mr.sharedMaterials = cadMats;
                        break;
                    }
                }
            }

            // Attach Controllers
            robotObj.AddComponent<KinematicOmniDrive>();
            robotObj.AddComponent<SimulationDashboard>();

            // 7. Livox Mid-360S Sensor Child (/World/base_link/mid360s)
            GameObject lidarObj = new GameObject("LivoxMid360S");
            lidarObj.transform.SetParent(robotObj.transform, false);
            lidarObj.transform.localPosition = (meta != null && meta.mid360LocalPositionUnity != null && meta.mid360LocalPositionUnity.Length == 3)
                ? new Vector3(meta.mid360LocalPositionUnity[0], meta.mid360LocalPositionUnity[1], meta.mid360LocalPositionUnity[2])
                : new Vector3(-0.257487f, 1.068127f, -0.146944f);
            lidarObj.transform.localRotation = Quaternion.identity;
            lidarObj.AddComponent<LivoxMid360Sim>();

            // Mid360 visual matching mid360s.usd (Cube + Sphere)
            Material lidarMat = CreateMat("Mat_Mid360", new Color(0.18f, 0.19f, 0.21f), 0.5f, 0.7f);
            GameObject lidarCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            lidarCube.name = "Cube";
            lidarCube.transform.SetParent(lidarObj.transform, false);
            lidarCube.transform.localPosition = new Vector3(0f, -0.015f, 0f);
            lidarCube.transform.localScale = new Vector3(0.065f, 0.03f, 0.065f);
            lidarCube.GetComponent<Renderer>().sharedMaterial = lidarMat;
            Object.DestroyImmediate(lidarCube.GetComponent<BoxCollider>());

            GameObject lidarSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            lidarSphere.name = "Sphere";
            lidarSphere.transform.SetParent(lidarObj.transform, false);
            lidarSphere.transform.localPosition = new Vector3(0f, 0.01f, 0f);
            lidarSphere.transform.localScale = new Vector3(0.055f, 0.045f, 0.055f);
            lidarSphere.GetComponent<Renderer>().sharedMaterial = lidarMat;
            Object.DestroyImmediate(lidarSphere.GetComponent<SphereCollider>());

            // 7.5 Lakibeam 1 2D LiDAR Child (/World/base_link/lakibeam1)
            // ROS base_link coordinates: x = -0.33m, y = 0.0m, z = 0.08m, yaw = 180 deg
            GameObject lakiObj = new GameObject("Lakibeam1");
            lakiObj.transform.SetParent(robotObj.transform, false);
            lakiObj.transform.localPosition = new Vector3(0.0f, 0.08f, -0.33f);
            lakiObj.transform.localRotation = Quaternion.Euler(0f, 180f, 0f);
            lakiObj.AddComponent<Lakibeam1Sim>();

            Material lakiMat = CreateMat("Mat_Lakibeam1", new Color(0.12f, 0.12f, 0.14f), 0.6f, 0.6f);
            GameObject lakiCyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            lakiCyl.name = "Cylinder";
            lakiCyl.transform.SetParent(lakiObj.transform, false);
            lakiCyl.transform.localPosition = Vector3.zero;
            lakiCyl.transform.localScale = new Vector3(0.08f, 0.04f, 0.08f);
            lakiCyl.GetComponent<Renderer>().sharedMaterial = lakiMat;
            Object.DestroyImmediate(lakiCyl.GetComponent<CapsuleCollider>());

            // 8. RealSense Sensor Child (Matching tracker_params.yaml: x=0.50m, z=1.00m, pitch=20deg, roll=90deg)
            GameObject rsObj = new GameObject("RealSenseD435i");
            rsObj.transform.SetParent(robotObj.transform, false);
            rsObj.transform.localPosition = new Vector3(0.0f, 1.00f, 0.50f);
            rsObj.transform.localRotation = Quaternion.Euler(-20.0f, 0f, -90.0f);
            Camera rsCam = rsObj.AddComponent<Camera>();
            rsCam.enabled = false;
            rsCam.fieldOfView = 58.0f;
            rsCam.nearClipPlane = 0.1f;
            rsCam.farClipPlane = 15.0f;
            rsObj.AddComponent<RealSenseDepthSim>();

            // 9. Cloth Shooter Child (/World/base_link/release_point)
            GameObject shooterObj = new GameObject("ClothShooter");
            shooterObj.transform.SetParent(robotObj.transform, false);
            shooterObj.transform.localPosition = (meta != null && meta.releasePointLocalPositionUnity != null && meta.releasePointLocalPositionUnity.Length == 3)
                ? new Vector3(meta.releasePointLocalPositionUnity[0], meta.releasePointLocalPositionUnity[1], meta.releasePointLocalPositionUnity[2])
                : new Vector3(0.002119f, 1.127392f, 0.557788f);
            shooterObj.transform.localRotation = Quaternion.identity;
            var shooter = shooterObj.AddComponent<SimpleClothShooter>();
            var shooterSettings = new SerializedObject(shooter);
            shooterSettings.FindProperty("clothPrefab").objectReferenceValue = clothPrefab;
            shooterSettings.ApplyModifiedPropertiesWithoutUndo();

            // Frame SceneView camera to show the robot and field clearly
            if (SceneView.lastActiveSceneView != null)
            {
                SceneView.lastActiveSceneView.drawGizmos = false;
                SceneView.lastActiveSceneView.LookAt(
                    AlignPointToMap(new Vector3(-2.6f, 0.6f, -1.5f)),
                    Quaternion.Euler(24f, -54f, 0f),
                    3.2f
                );
                SceneView.lastActiveSceneView.Repaint();
            }

            // Save Scene
            string scenePath = "Assets/Scenes/NHK2026_SimScene.unity";
            EditorSceneManager.SaveScene(scene, scenePath);
            ScenePhysicsBaker.BakeAndSave(scene);
            Debug.Log("[SimSetupEditor] Exact Isaac Sim (main.usd) scene built, aligned to nhk2026_map.json, and saved!");
        }
    }
}
#endif
