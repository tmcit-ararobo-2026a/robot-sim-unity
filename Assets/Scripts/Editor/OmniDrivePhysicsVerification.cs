using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using RobotSim.Control;
using RobotSim.Environment;

namespace RobotSim.Editor
{
    // Run in Play mode. Each case owns an isolated PhysX scene; the match is untouched.
    public static class OmniDrivePhysicsVerification
    {
        [Serializable]
        public sealed class Report
        {
            public bool passed;
            public int assertions;
            public string error;
            public float forwardDistance;
            public float lateralDistance;
            public float wallStopPosition;
            public float pushedDistance;
            public int movableFieldPropsTested;
            public int bucketsSupported;
            public int centralStepCasesPassed;
            public int movablePlatformsTested;
            public float speedAfterHalfSecond;
            public float speedAfterQuarterSecond;
            public float speedAfterTenthSecondBraking;
            public float angularSpeedAfterQuarterSecond;
            public float yawStopTime;
            public float yawReverseTime;
            public float brakingDistance;
        }

        private static Report _report;

        [MenuItem("RobotSim/Verify Rigidbody Drive (Play Mode)")]
        public static void RunFromMenu() => Debug.Log(JsonUtility.ToJson(Run(), true));

        public static Report Run()
        {
            if (!Application.isPlaying) throw new InvalidOperationException("Enter Play mode before verifying the drive.");
            _report = new Report();
            try
            {
                VerifyGravityAndAirborneDrive();
                VerifyMovementAndBraking();
                VerifyWallAndPushing();
                VerifyFieldProps();
                VerifyBucketPlatforms();
                VerifyCentralStep();
                _report.passed = true;
            }
            catch (Exception ex) { _report.error = ex.ToString(); }
            finally
            {
                // Restore the sensor snapshot to the actual player after fixture teardown.
                foreach (var drive in UnityEngine.Object.FindObjectsByType<KinematicOmniDrive>(FindObjectsSortMode.None))
                    drive.SendMessage("LateUpdate");
            }
            System.IO.Directory.CreateDirectory("Temp");
            System.IO.File.WriteAllText("Temp/OmniDrivePhysicsVerification.json", JsonUtility.ToJson(_report, true));
            return _report;
        }

        private static void Check(bool condition, string message)
        {
            _report.assertions++;
            if (!condition) throw new InvalidOperationException(message);
        }

        private static void VerifyGravityAndAirborneDrive()
        {
            using (var f = new Fixture())
            {
                Check(!f.Body.isKinematic && f.Body.useGravity, "Drive must be dynamic with gravity enabled.");
                Check(f.Body.collisionDetectionMode == CollisionDetectionMode.ContinuousDynamic, "Continuous collision detection is missing.");
                float startY = f.Body.position.y;
                f.Step(10, new Vector3(2f, 0f, 0f), 1f);
                Check(f.Body.position.y < startY - 0.03f, "Gravity did not lower the airborne robot.");
                Check(Mathf.Abs(f.Body.position.z) < 0.001f && Mathf.Abs(f.Body.angularVelocity.y) < 0.001f,
                    "The drive produced traction while airborne.");
                Check(KinematicOmniDrive.GetMotionState().LinearAccelerationBase.y < -9f, "Snapshot omitted free-fall acceleration.");
                f.Step(140);
                Check(f.Drive.IsGrounded && Mathf.Abs(f.Body.position.y) < 0.04f, "Robot did not settle on the floor.");
            }
        }

        private static void VerifyMovementAndBraking()
        {
            using (var f = new Fixture())
            {
                f.Step(150);
                Vector3 start = f.Body.position;
                f.Step(1, new Vector3(2f, 0f, 0f));
                Check(f.Body.linearVelocity.z <= 0.04f, "Forward command caused an abrupt speed jump.");
                f.Step(24, new Vector3(2f, 0f, 0f));
                _report.speedAfterQuarterSecond = f.Body.linearVelocity.z;
                Check(_report.speedAfterQuarterSecond > 0.85f && _report.speedAfterQuarterSecond < 1.15f,
                    "Forward response is too sluggish or too abrupt during the first quarter second.");
                f.Step(25, new Vector3(2f, 0f, 0f));
                _report.speedAfterHalfSecond = f.Body.linearVelocity.z;
                Check(_report.speedAfterHalfSecond > 1.85f && _report.speedAfterHalfSecond < 2.05f,
                    "Forward response is too sluggish or too abrupt during the first half second.");
                f.Step(71, new Vector3(2f, 0f, 0f));
                _report.forwardDistance = f.Body.position.z - start.z;
                Check(_report.forwardDistance > 1f, $"Forward command did not move the rigidbody: grounded={f.Drive.IsGrounded}, velocity={f.Body.linearVelocity}, position={f.Body.position}, dt={Time.fixedDeltaTime}.");
                Check(f.Body.linearVelocity.magnitude <= 2.05f, "Linear speed limit was exceeded.");
                float stopStart = f.Body.position.z;
                f.Step(10);
                _report.speedAfterTenthSecondBraking = f.Body.linearVelocity.z;
                Check(_report.speedAfterTenthSecondBraking > 0.5f, "Braking removed momentum too abruptly.");
                f.Step(40);
                Check(Mathf.Abs(f.Body.linearVelocity.z) < 0.05f, "Zero command failed to brake.");
                Check(f.Body.position.z > stopStart, "Braking teleported/stopped the robot instantly.");
                _report.brakingDistance = f.Body.position.z - stopStart;

                start = f.Body.position;
                f.Step(100, new Vector3(0f, 1f, 0f));
                _report.lateralDistance = start.x - f.Body.position.x;
                Check(_report.lateralDistance > 0.5f, "ROS positive lateral command did not move left.");
                f.Step(50);
                f.Step(25, Vector3.zero, 3.14f);
                _report.angularSpeedAfterQuarterSecond = Mathf.Abs(f.Body.angularVelocity.y);
                Check(_report.angularSpeedAfterQuarterSecond > 1.5f && _report.angularSpeedAfterQuarterSecond < 2.2f,
                    "Yaw response is too sluggish or too abrupt during the first quarter second.");
                f.Step(75, Vector3.zero, 1f);
                Check(f.Body.angularVelocity.y < -0.1f, "ROS positive yaw did not rotate counterclockwise.");
                var state = KinematicOmniDrive.GetMotionState();
                Check(state.AngularVelocityRadZ > 0.1f, "Snapshot used the wrong angular sign.");
                Check(Vector3.Distance(state.Position, f.Body.position) < 0.0001f, "Snapshot position differs from the physical body.");
                Check(Vector3.Distance(state.LinearVelocityWorld, f.Body.linearVelocity) < 0.0001f, "Snapshot velocity differs from the physical body.");

                f.Step(60, Vector3.zero, 3.14f);
                Check(-f.Body.angularVelocity.y > 2.9f, "Yaw did not reach the configured full angular speed.");
                for (int step = 1; step <= 50; step++)
                {
                    f.Step(1);
                    if (Mathf.Abs(f.Body.angularVelocity.y) < 0.05f)
                    {
                        _report.yawStopTime = step * 0.01f;
                        break;
                    }
                }
                Check(_report.yawStopTime > 0 && _report.yawStopTime <= 0.35f,
                    "Yaw braking did not respond within 0.35 seconds.");
                f.Step(80, Vector3.zero, 3.14f);
                for (int step = 1; step <= 60; step++)
                {
                    f.Step(1, Vector3.zero, -3.14f);
                    if (f.Body.angularVelocity.y > 0.5f)
                    {
                        _report.yawReverseTime = step * 0.01f;
                        break;
                    }
                }
                Check(_report.yawReverseTime > 0 && _report.yawReverseTime <= 0.45f,
                    "Yaw reversal did not respond within 0.45 seconds.");

                f.Step(50);
                f.Step(60, new Vector3(1f, 0f, 0f));
                System.Threading.Thread.Sleep(300);
                f.StepWithoutCommand(50);
                Check(f.Body.linearVelocity.magnitude < 0.05f, "Stale command did not brake after the timeout.");
            }
        }

        private static void VerifyWallAndPushing()
        {
            using (var f = new Fixture())
            {
                f.Box("Thin wall", new Vector3(0, 1, 2), new Vector3(10, 2, 0.03f));
                f.Step(150);
                f.Step(300, new Vector3(2f, 0f, 0f));
                _report.wallStopPosition = f.Body.position.z;
                Check(f.Body.position.z > 1f && f.Body.position.z < 1.62f, "Robot passed through the thin wall.");
                Check(Mathf.Abs(f.Body.linearVelocity.z) < 0.1f, "Wall contact did not stop physical forward velocity.");
                Check(Mathf.Abs(KinematicOmniDrive.GetMotionState().Position.z - f.Body.position.z) < 0.0001f,
                    "Sensor position continued through the wall.");
            }
            using (var f = new Fixture())
            {
                var box = f.Box("Pushable body", new Vector3(0, 0.2f, 1.2f), new Vector3(0.4f, 0.4f, 0.4f));
                var pushed = box.AddComponent<Rigidbody>();
                pushed.mass = 5f;
                f.Step(150);
                float start = pushed.position.z;
                f.Step(150, new Vector3(1f, 0f, 0f));
                _report.pushedDistance = pushed.position.z - start;
                Check(_report.pushedDistance > 0.3f, "Dynamic robot could not push another rigidbody.");
            }
        }

        private static void VerifyFieldProps()
        {
            string[] names = { "desk1_red", "chair_red", "bucket1_red", "flag_red" };
            float[] masses = { 8f, 5f, 0.6f, 10f };
            for (int i = 0; i < names.Length; i++)
            {
                var source = GameObject.Find(names[i]);
                Check(source != null, "Field prop missing from this scene: " + names[i]);
                Mesh mesh = source.GetComponentInChildren<MeshFilter>().sharedMesh;
                using (var f = new Fixture())
                {
                    Rigidbody prop = f.FieldProp(names[i], mesh, masses[i]);
                    Check(!prop.isKinematic && prop.useGravity && prop.constraints == RigidbodyConstraints.None,
                        names[i] + " is not a freely moving body.");
                    Check(Mathf.Abs(prop.mass - masses[i]) < 0.001f, names[i] + " has the wrong mass.");
                    foreach (var collider in prop.GetComponentsInChildren<Collider>())
                        Check(!collider.enabled || !(collider is MeshCollider), "A non-convex field mesh is attached to a dynamic body.");
                    Check(FieldPropPhysics.MakeMovable(prop.gameObject, masses[i]) == prop,
                        "Repeated field setup replaced an existing body.");
                    f.Step(150);
                    Vector3 before = prop.position;
                    f.Step(180, new Vector3(1f, 0f, 0f));
                    Check(prop.position.z > before.z + 0.15f, names[i] + " did not move when hit by the robot.");
                    _report.movableFieldPropsTested++;
                }
            }
            foreach (string fixedName in new[] { "field_base" })
            {
                Check(!FieldPropPhysics.TryGetMass(fixedName, 8, 5, 0.6f, 10, 3, out _),
                    "Fixed structure was classified as movable: " + fixedName);
                var fixedObject = GameObject.Find(fixedName);
                Check(fixedObject != null && fixedObject.GetComponent<Rigidbody>() == null,
                    "Fixed structure acquired a Rigidbody: " + fixedName);
            }
            var fieldBase = GameObject.Find("field_base");
            var groundPhysics = fieldBase.GetComponent<FieldGroundPhysics>();
            Check(groundPhysics != null && groundPhysics.FloorCollider != null &&
                groundPhysics.FloorCollider.gameObject == fieldBase && groundPhysics.FloorCollider.attachedRigidbody == null,
                "The fixed floor collider must belong to field_base.");
            // Cross the paint/tile edge that previously stopped the player short of desk1.
            bool first = Physics.Raycast(new Vector3(3.86f, 0.1f, -3.08f), Vector3.down, out RaycastHit a,
                0.5f, ~0, QueryTriggerInteraction.Ignore);
            bool second = Physics.Raycast(new Vector3(3.86f, 0.1f, -3.00f), Vector3.down, out RaycastHit b,
                0.5f, ~0, QueryTriggerInteraction.Ignore);
            Check(first && second, "The desk approach has a gap in the supporting floor.");
            Check(Mathf.Abs(a.point.y - b.point.y) < 0.0001f, "Floor paint still creates a collision step.");
        }

        private static void VerifyBucketPlatforms()
        {
            string[] platforms = { "bucket2_platform_red", "bucket2_platform_blue", "bucket3_platform_red", "bucket3_platform" };
            string[] buckets = { "bucket2_red", "bucket2_blue", "bucket3_red", "bucket3_blue" };
            for (int i = 0; i < platforms.Length; i++)
            {
                Mesh platformMesh = GameObject.Find(platforms[i]).GetComponentInChildren<MeshFilter>().sharedMesh;
                Mesh bucketMesh = GameObject.Find(buckets[i]).GetComponentInChildren<MeshFilter>().sharedMesh;
                using (var f = new Fixture())
                {
                    Rigidbody platform = f.FieldProp(platforms[i], platformMesh, 3f, 0f, 0.02f, 2f);
                    Check(!platform.isKinematic && platform.useGravity && Mathf.Abs(platform.mass - 3f) < 0.001f,
                        platforms[i] + " is not a dynamic platform with the configured mass.");
                    float initialTop = platform.position.y + platformMesh.bounds.size.y * 0.5f;
                    Rigidbody bucket = f.FieldProp(buckets[i], bucketMesh, 0.6f, initialTop, 0.3f, 2f);
                    f.Step(200);
                    float top = platform.position.y + platformMesh.bounds.size.y * 0.5f;
                    float bottom = bucket.position.y - bucketMesh.bounds.size.y * 0.5f;
                    Check(bottom >= top - 0.01f && bottom <= top + 0.025f,
                        $"{buckets[i]} fell into its platform: bottom={bottom}, top={top}.");
                    Check(bucket.linearVelocity.magnitude < 0.05f, buckets[i] + " did not settle on the platform.");
                    Check(Vector3.Dot(bucket.rotation * Vector3.up, Vector3.up) > 0.98f,
                        buckets[i] + " tipped during a straight drop onto its platform.");
                    _report.bucketsSupported++;
                    Vector3 before = platform.position;
                    f.Step(300, new Vector3(0.6f, 0, 0));
                    Check(platform.position.z > before.z + 0.15f,
                        platforms[i] + " did not move when pushed by the robot.");
                    _report.movablePlatformsTested++;
                }
            }
        }

        private static void VerifyCentralStep()
        {
            Mesh fieldMesh = GameObject.Find("field_base").GetComponent<MeshFilter>().sharedMesh;
            float[] yaw = { 0f, 180f, 45f, 225f, 0f, 0f };
            float[] side = { -1f, 1f, -1f, 1f, -1f, 1f };
            Vector3[] commands = { new Vector3(2, 0, 0), new Vector3(2, 0, 0),
                new Vector3(2, 0, 0), new Vector3(2, 0, 0), new Vector3(2, 1, 0), new Vector3(-2, -1, 0) };
            for (int i = 0; i < yaw.Length; i++)
            {
                using (var f = new Fixture())
                {
                    f.FieldStructure(fieldMesh);
                    f.Body.position = new Vector3(0, 2, side[i] * 1.5f);
                    f.Body.rotation = Quaternion.Euler(0, yaw[i], 0);
                    f.Step(150);
                    float restY = f.Body.position.y;
                    float highestY = restY;
                    for (int step = 0; step < 300; step++)
                    {
                        f.Step(1, commands[i]);
                        highestY = Mathf.Max(highestY, f.Body.position.y);
                    }
                    Check(side[i] * f.Body.position.z > 0.65f,
                        $"Robot crossed the central step in case {i}: {f.Body.position}.");
                    Check(highestY < restY + 0.035f,
                        $"Robot climbed the central step in case {i}: rise={highestY - restY}.");
                    Check(f.RayDown(new Vector3(0, 1, 0), out RaycastHit top) && top.normal.y > 0.99f &&
                        Mathf.Abs(top.point.y - 0.2f) < 0.001f, "Central step has no upward-facing solid top.");
                    _report.centralStepCasesPassed++;
                }
            }
        }

        private sealed class Fixture : IDisposable
        {
            private readonly Scene _scene;
            private readonly PhysicsScene _physics;
            public readonly Rigidbody Body;
            public readonly KinematicOmniDrive Drive;
            private readonly GameObject _ground;
            private const float Dt = 0.01f;

            public Fixture()
            {
                _scene = SceneManager.CreateScene("Drive verification " + Guid.NewGuid(),
                    new CreateSceneParameters(LocalPhysicsMode.Physics3D));
                _physics = _scene.GetPhysicsScene();
                _ground = Box("Ground", new Vector3(0, -0.1f, 0), new Vector3(30, 0.2f, 30));
                var robot = new GameObject("Physics verification robot");
                SceneManager.MoveGameObjectToScene(robot, _scene);
                robot.transform.position = new Vector3(0, 2, 0);
                var collider = robot.AddComponent<BoxCollider>();
                collider.size = new Vector3(0.75f, 1f, 0.85f);
                collider.center = new Vector3(0, 0.5f, 0);
                Body = robot.AddComponent<Rigidbody>();
                Body.mass = 25f;
                Body.linearDamping = 0.5f;
                Body.angularDamping = 1f;
                Drive = robot.AddComponent<KinematicOmniDrive>();
            }

            public GameObject Box(string name, Vector3 position, Vector3 size)
            {
                var obj = new GameObject(name);
                SceneManager.MoveGameObjectToScene(obj, _scene);
                obj.transform.position = position;
                obj.AddComponent<BoxCollider>().size = size;
                return obj;
            }

            public Rigidbody FieldProp(string name, Mesh mesh, float mass, float supportHeight = 0f,
                float dropHeight = 0.02f, float forwardPosition = 1.2f)
            {
                var obj = new GameObject(name);
                SceneManager.MoveGameObjectToScene(obj, _scene);
                Bounds bounds = mesh.bounds;
                obj.transform.position = new Vector3(-bounds.center.x, -bounds.min.y + supportHeight + dropHeight,
                    forwardPosition - bounds.center.z);
                obj.AddComponent<MeshFilter>().sharedMesh = mesh;
                obj.AddComponent<MeshCollider>().sharedMesh = mesh;
                Vector3 localVertex = mesh.vertices[0];
                Vector3 before = obj.transform.TransformPoint(localVertex);
                Rigidbody body = FieldPropPhysics.MakeMovable(obj, mass);
                Check(Vector3.Distance(before, obj.transform.TransformPoint(localVertex)) < 0.00001f,
                    "Adding field physics changed the rendered geometry position.");
                return body;
            }

            public void FieldStructure(Mesh mesh)
            {
                var obj = new GameObject("Field collision regression fixture");
                SceneManager.MoveGameObjectToScene(obj, _scene);
                obj.transform.position = new Vector3(-mesh.bounds.center.x, -mesh.bounds.min.y, -mesh.bounds.center.z);
                obj.AddComponent<MeshCollider>().sharedMesh = mesh;
                obj.AddComponent<FieldGroundPhysics>().AlignGround(_ground.GetComponent<BoxCollider>());
            }

            public bool RayDown(Vector3 origin, out RaycastHit hit) =>
                _physics.Raycast(origin, Vector3.down, out hit, 2f);

            public void Step(int count, Vector3 linearRos = default, float angularRosZ = 0f)
            {
                for (int i = 0; i < count; i++)
                {
                    Drive.SetVelocityCommand(linearRos, angularRosZ);
                    StepWithoutCommand(1);
                }
            }

            public void StepWithoutCommand(int count)
            {
                for (int i = 0; i < count; i++)
                {
                    Drive.SendMessage("FixedUpdate");
                    _physics.Simulate(Dt);
                    Drive.SendMessage("LateUpdate");
                }
            }

            public void Dispose()
            {
                foreach (var root in _scene.GetRootGameObjects()) UnityEngine.Object.DestroyImmediate(root);
                SceneManager.UnloadSceneAsync(_scene);
            }
        }
    }
}
