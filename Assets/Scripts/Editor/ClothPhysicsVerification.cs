using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using RobotSim.Control;
using RobotSim.Shooter;

namespace RobotSim.Editor
{
    // Exercise the public firing path, including its sensor snapshot, in isolated scenes.
    public static class ClothPhysicsVerification
    {
        [MenuItem("RobotSim/Verify Fired Cloth Collisions (Play Mode)")]
        public static void RunFromMenu() => Debug.Log(Run());

        public static string Run()
        {
            if (!Application.isPlaying) throw new InvalidOperationException("Enter Play mode first.");
            var clothPrefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefab/cloth.prefab");
            Require(clothPrefab != null, "Cloth prefab is missing.");
            var original = SceneManager.GetActiveScene();
            var player = UnityEngine.Object.FindFirstObjectByType<KinematicOmniDrive>();
            var field = GameObject.Find("Field");
            var desk = field.transform.Find("desk1_red").gameObject;
            var fieldBucket = field.transform.Find("bucket1_red").gameObject;
            var movingBucket = GameObject.Find("Opponent_Moving_Bucket");
            Require(movingBucket != null, "Opponent bucket is missing.");
            var report = new System.Collections.Generic.List<string>();
            try
            {
                for (int test = 0; test < 11; test++)
                {
                    var scene = SceneManager.CreateScene("Fired cloth verification " + test,
                        new CreateSceneParameters(LocalPhysicsMode.Physics3D));
                    SceneManager.SetActiveScene(scene);
                    try
                    {
                        var robot = new GameObject("Launch pose");
                        var drive = robot.AddComponent<KinematicOmniDrive>();
                        drive.enabled = false;
                        var shooter = new GameObject("Test shooter").AddComponent<SimpleClothShooter>();
                        shooter.enabled = false;
                        var settings = new SerializedObject(shooter);
                        settings.FindProperty("clothPrefab").objectReferenceValue = clothPrefab;
                        settings.ApplyModifiedPropertiesWithoutUndo();
                        Vector3 spawn;
                        Vector3 velocity;
                        GameObject obstacle;
                        float surface = 0f;
                        float bucketBottom = 0f;
                        Quaternion rotation = Quaternion.identity;
                        if (test < 2)
                        {
                            obstacle = new GameObject("Thin wall");
                            obstacle.AddComponent<BoxCollider>().size = new Vector3(5f, 20f, 0.01f);
                            if (test == 1)
                            {
                                var body = obstacle.AddComponent<Rigidbody>();
                                body.useGravity = false;
                                body.mass = 25f;
                                body.constraints = RigidbodyConstraints.FreezeAll;
                            }
                            spawn = new Vector3(0, 1.52f, -0.6f);
                            velocity = new Vector3(0, 0, 20f);
                        }
                        else if (test == 2)
                        {
                            obstacle = UnityEngine.Object.Instantiate(desk);
                            obstacle.transform.position = new Vector3(0, obstacle.transform.position.y, 0);
                            var bounds = obstacle.GetComponentInChildren<Renderer>().bounds;
                            obstacle.transform.position += Vector3.up * -bounds.min.y;
                            bounds = obstacle.GetComponentInChildren<Renderer>().bounds;
                            surface = bounds.max.y;
                            var floor = new GameObject("Floor").AddComponent<BoxCollider>();
                            floor.size = new Vector3(5f, 0.1f, 5f);
                            floor.center = new Vector3(0, -0.05f, 0);
                            spawn = new Vector3(bounds.center.x, surface + 0.8f, bounds.center.z);
                            velocity = new Vector3(0, -20f, 0);
                        }
                        else
                        {
                            // Use the saved buckets' actual bottom and wall colliders.
                            obstacle = UnityEngine.Object.Instantiate(test < 7 ? fieldBucket : movingBucket);
                            foreach (var behaviour in obstacle.GetComponentsInChildren<MonoBehaviour>())
                                UnityEngine.Object.DestroyImmediate(behaviour);
                            obstacle.transform.position = Vector3.zero;
                            var bucketBody = obstacle.GetComponent<Rigidbody>();
                            if (bucketBody != null)
                            {
                                bucketBody.isKinematic = true;
                                bucketBody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
                            }
                            Bounds bounds = obstacle.GetComponentInChildren<Renderer>().bounds;
                            surface = bounds.max.y;
                            bucketBottom = bounds.min.y;
                            spawn = new Vector3(bounds.center.x, surface + 0.5f, bounds.center.z);
                            velocity = new Vector3(0, -8f, 0);
                            int pose = (test - 3) % 4;
                            rotation = pose == 0 ? Quaternion.identity : pose == 1 ? Quaternion.Euler(0, 45, 0)
                                : pose == 2 ? Quaternion.Euler(35, 45, 25) : Quaternion.Euler(90, 45, 0);
                        }
                        var robotBody = robot.GetComponent<Rigidbody>();
                        robotBody.rotation = rotation;
                        robotBody.position = spawn - rotation * new Vector3(0, 1.52f, 0.9f);
                        drive.SendMessage("LateUpdate");
                        shooter.ShootCloth(0.4f, Quaternion.Inverse(rotation) * velocity);
                        shooter.SendMessage("SyncClothVisuals");
                        Rigidbody cloth = null;
                        foreach (var root in scene.GetRootGameObjects())
                            if (root.CompareTag("DustCloth")) cloth = root.GetComponent<Rigidbody>();
                        Require(cloth != null && !cloth.isKinematic, "Firing did not create a dynamic cloth.");
                        Require(cloth.GetComponent<MeshFilter>().sharedMesh == clothPrefab.GetComponent<MeshFilter>().sharedMesh,
                            "Fired cloth did not use the prefab mesh asset.");
                        Require(Mathf.Approximately(cloth.linearDamping, clothPrefab.GetComponent<Rigidbody>().linearDamping),
                            "Firing overrode the prefab physics settings.");
                        Require(!cloth.GetComponent<Collider>().isTrigger, "Fired cloth only has a trigger.");
                        Require(Vector3.Distance(cloth.linearVelocity, velocity) < 0.001f, "Launch velocity was lost.");
                        Physics.SyncTransforms();
                        var physics = scene.GetPhysicsScene();
                        float furthest = cloth.position.z;
                        for (int step = 0; step < 60; step++)
                        {
                            physics.Simulate(0.02f);
                            furthest = Mathf.Max(furthest, cloth.position.z);
                        }
                        Vector3 stopped = cloth.position;
                        shooter.SendMessage("SyncClothVisuals");
                        Require(Vector3.Distance(cloth.position, stopped) < 0.001f, "Visual sync overwrote physics.");
                        if (test < 2) Require(furthest < 0f, "Cloth passed through a 1 cm wall at 20 m/s.");
                        else if (test == 2) Require(stopped.y > surface, "Cloth fell through the actual field desk.");
                        else
                        {
                            var box = cloth.GetComponent<BoxCollider>();
                            Vector3 half = box.size * 0.5f;
                            float halfHeight = Mathf.Abs((cloth.rotation * Vector3.right).y) * half.x
                                + Mathf.Abs((cloth.rotation * Vector3.up).y) * half.y
                                + Mathf.Abs((cloth.rotation * Vector3.forward).y) * half.z;
                            float centerY = (cloth.position + cloth.rotation * box.center).y;
                            Require(centerY + halfHeight < surface - 0.005f,
                                "Cloth caught on the bucket rim, case " + test + ".");
                            Require(centerY - halfHeight >= bucketBottom - 0.015f,
                                "Cloth passed through the bucket bottom, case " + test + ".");
                            Require(new Vector2(stopped.x, stopped.z).magnitude < 0.1f,
                                "Cloth escaped the bucket walls, case " + test + ".");
                        }
                        Require(Vector3.Distance(SimpleClothShooter.ActiveClothWorldPos, stopped) < 0.001f,
                            "Sensor position diverged from the physical cloth.");
                        shooter.ResetCloths();
                        Require(!SimpleClothShooter.IsClothFlying, "Reset left a flying cloth snapshot.");
                        string label = test == 0 ? "Static wall" : test == 1 ? "Dynamic body" : test == 2 ? "Field desk"
                            : (test < 7 ? "Field bucket" : "Opponent bucket") + " pose " + (test - 3) % 4;
                        report.Add(label + ": PASS at " + velocity.magnitude + " m/s, position=" + stopped.ToString("F4"));
                    }
                    finally
                    {
                        // Destroy now so fixture scripts never run Start in the match.
                        foreach (var root in scene.GetRootGameObjects()) UnityEngine.Object.DestroyImmediate(root);
                        SceneManager.SetActiveScene(original);
                        SceneManager.UnloadSceneAsync(scene);
                    }
                }
            }
            finally
            {
                SceneManager.SetActiveScene(original);
                if (player != null) player.SendMessage("LateUpdate");
            }
            return string.Join("\n", report);
        }

        private static void Require(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }
    }
}
