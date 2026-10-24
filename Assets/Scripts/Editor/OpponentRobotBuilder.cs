using System;
using UnityEngine;
using UnityEditor;
using RobotSim.Opponent;

namespace RobotSim.Editor
{
    public static class OpponentRobotBuilder
    {
        [MenuItem("RoboSim/Create or Update Opponent Robot (Rule 2026 Compliant)")]
        public static void CreateOrUpdateOpponentRobot()
        {
            const string RobotName = "OpponentRobot";
            GameObject robot = GameObject.Find(RobotName);
            if (robot != null)
            {
                Undo.DestroyObjectImmediate(robot);
            }

            robot = new GameObject(RobotName);
            Undo.RegisterCreatedObjectUndo(robot, "Create Opponent Robot");

            // 初期位置: 青ゾーンのスタート位置 (補充机 desk_replenish_blue: X=-4.47, Z=1.08 付近)
            robot.transform.position = new Vector3(-3.6f, 0.05f, 1.2f);
            robot.transform.rotation = Quaternion.Euler(0f, 180f, 0f);

            // 1. Rigidbody & Controller
            Rigidbody rb = robot.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

            OpponentRobotController controller = robot.AddComponent<OpponentRobotController>();

            // マテリアルの準備
            Material blueChassisMat = CreateColorMaterial("Mat_Opponent_Blue", new Color(0.12f, 0.35f, 0.85f, 1.0f));
            Material chairWoodMat = CreateColorMaterial("Mat_Opponent_Wood", new Color(0.78f, 0.55f, 0.35f, 1.0f));
            Material steelPipeMat = CreateColorMaterial("Mat_Opponent_Steel", new Color(0.3f, 0.3f, 0.3f, 1.0f));
            Material mascotMat = CreateColorMaterial("Mat_Opponent_Mascot", new Color(0.95f, 0.55f, 0.1f, 1.0f));
            Material bucketMat = CreateTransparentMaterial("Mat_Opponent_PolyBucket", new Color(0.85f, 0.92f, 1.0f, 0.45f));

            // 2. ベース車体 (1000mm x 1000mm x 350mm) - ルール3.2.2準拠
            GameObject chassis = GameObject.CreatePrimitive(PrimitiveType.Cube);
            chassis.name = "Chassis_Base";
            chassis.transform.SetParent(robot.transform, false);
            chassis.transform.localPosition = new Vector3(0f, 0.175f, 0f);
            chassis.transform.localScale = new Vector3(1.0f, 0.35f, 1.0f);
            chassis.GetComponent<Renderer>().sharedMaterial = blueChassisMat;

            // 3. 教室用椅子 (新JIS 6号: 座面幅360x奥行400x高460, 全体高807mm) - ルール3.1.2 / 5.5準拠
            GameObject chairRoot = new GameObject("Classroom_Chair_JIS6");
            chairRoot.transform.SetParent(robot.transform, false);
            chairRoot.transform.localPosition = new Vector3(0f, 0.35f, -0.1f); // 車体上面

            // 4本の脚
            for (int i = 0; i < 4; i++)
            {
                float xOff = (i % 2 == 0) ? -0.16f : 0.16f;
                float zOff = (i / 2 == 0) ? -0.18f : 0.18f;
                GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                leg.name = $"Chair_Leg_{i}";
                leg.transform.SetParent(chairRoot.transform, false);
                leg.transform.localPosition = new Vector3(xOff, 0.23f, zOff);
                leg.transform.localScale = new Vector3(0.03f, 0.23f, 0.03f);
                leg.GetComponent<Renderer>().sharedMaterial = steelPipeMat;
            }

            // 座面 (360mm x 400mm x 20mm, 高さ460mm)
            GameObject seat = GameObject.CreatePrimitive(PrimitiveType.Cube);
            seat.name = "Chair_Seat";
            seat.transform.SetParent(chairRoot.transform, false);
            seat.transform.localPosition = new Vector3(0f, 0.46f, 0f);
            seat.transform.localScale = new Vector3(0.36f, 0.02f, 0.40f);
            seat.GetComponent<Renderer>().sharedMaterial = chairWoodMat;

            // 背もたれ (幅360mm x 高180mm, 最上部807mm)
            GameObject backrest = GameObject.CreatePrimitive(PrimitiveType.Cube);
            backrest.name = "Chair_Backrest";
            backrest.transform.SetParent(chairRoot.transform, false);
            backrest.transform.localPosition = new Vector3(0f, 0.72f, -0.19f);
            backrest.transform.localScale = new Vector3(0.36f, 0.18f, 0.02f);
            backrest.GetComponent<Renderer>().sharedMaterial = chairWoodMat;

            // 4. マスコット (350mm x 350mm x 650mm) - ルール3.1.3「300x300x600mm以上」準拠
            GameObject mascot = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            mascot.name = "Team_Mascot";
            mascot.transform.SetParent(robot.transform, false);
            // 椅子座面(0.35 + 0.46 = 0.81m)に着席
            mascot.transform.localPosition = new Vector3(0f, 0.81f + 0.325f, -0.05f);
            mascot.transform.localScale = new Vector3(0.35f, 0.325f, 0.35f); // 全高 0.65m
            mascot.GetComponent<Renderer>().sharedMaterial = mascotMat;

            // 5. 移動バケツ支柱 (Bucket Post)
            // ルール3.2.3.b: バケツ上面は床面から 1200mm〜2100mm。
            // ルール3.2.3.c: ロボット・マスコットのいかなる部分も競技中にバケツの2L目盛りより上に出てはならないため、
            // マスコット頭頂部(約1.50m)より高い 1.80m に上面を配置して完全適合。
            const float BucketTopHeightFromFloor = 1.80f;
            const float BucketHeight = 0.255f;   // PO-24A規格 255mm
            const float BucketRadius = 0.1365f;  // 直径273mm / 2
            float bucketCenterY = BucketTopHeightFromFloor - (BucketHeight * 0.5f);

            GameObject post = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            post.name = "Bucket_Post";
            post.transform.SetParent(robot.transform, false);
            float postBottom = 0.35f;
            float postTop = BucketTopHeightFromFloor - BucketHeight;
            float postHeight = postTop - postBottom;
            post.transform.localPosition = new Vector3(0f, postBottom + (postHeight * 0.5f), 0.35f); // 機体前方
            post.transform.localScale = new Vector3(0.05f, postHeight * 0.5f, 0.05f);
            post.GetComponent<Renderer>().sharedMaterial = steelPipeMat;

            // 6. 移動バケツ (エンテック PO-24A 規格: 直径273mm x 高さ255mm)
            GameObject bucket = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            bucket.name = "Opponent_Moving_Bucket";
            bucket.transform.SetParent(robot.transform, false);
            bucket.transform.localPosition = new Vector3(0f, bucketCenterY, 0.35f);
            bucket.transform.localScale = new Vector3(BucketRadius * 2f, BucketHeight * 0.5f, BucketRadius * 2f);
            bucket.GetComponent<Renderer>().sharedMaterial = bucketMat;

            // 真値TFパブリッシャー
            OpponentGroundTruthPublisher gtPub = bucket.AddComponent<OpponentGroundTruthPublisher>();

            // 7. バケツ開口部 & ゴールトリガー
            GameObject goalTriggerObj = new GameObject("Goal_Trigger");
            goalTriggerObj.transform.SetParent(bucket.transform, false);
            // バケツ上面内側
            goalTriggerObj.transform.localPosition = new Vector3(0f, 0.8f, 0f);
            SphereCollider sc = goalTriggerObj.AddComponent<SphereCollider>();
            sc.isTrigger = true;
            sc.radius = 0.85f; // バケツ内径に合わせる

            OpponentBucketGoalDetector goalDetector = goalTriggerObj.AddComponent<OpponentBucketGoalDetector>();

            Selection.activeGameObject = robot;
            EditorUtility.SetDirty(robot);
            Debug.Log($"<color=cyan>[OpponentRobotBuilder]</color> 2026公式ルール準拠の相手ロボットを生成しました! (バケツ上面高: {BucketTopHeightFromFloor:F2}m, 車体: 1.0x1.0m, 椅子: 新JIS6号, マスコット: 0.35x0.65m)");
        }

        private static Material CreateColorMaterial(string name, Color color)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            Material mat = new Material(shader)
            {
                name = name,
                color = color
            };
            return mat;
        }

        private static Material CreateTransparentMaterial(string name, Color color)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            Material mat = new Material(shader)
            {
                name = name,
                color = color
            };
            if (mat.HasProperty("_Mode")) mat.SetFloat("_Mode", 3); // Transparent
            if (mat.HasProperty("_Surface")) mat.SetFloat("_Surface", 1); // URP Transparent
            return mat;
        }
    }
}
