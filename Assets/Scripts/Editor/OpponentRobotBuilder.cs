using System;
using UnityEngine;
using UnityEditor;
using RobotSim.Environment;
using RobotSim.Opponent;

namespace RobotSim.Editor
{
    public static class OpponentRobotBuilder
    {
        [MenuItem("RoboSim/Create or Update Opponent Robot (Rule 2026 Compliant)")]
        public static void CreateOrUpdateOpponentRobot()
        {
            const string RobotName = "OpponentRobot";
            GameObject existing = GameObject.Find(RobotName);
            if (existing != null)
            {
                Undo.DestroyObjectImmediate(existing);
            }

            GameObject robot = new GameObject(RobotName);
            Undo.RegisterCreatedObjectUndo(robot, "Create Opponent Robot");

            // 初期位置: 青ゾーンのスタートゾーン (desk_replenish_blue: X=-1.06, Z=-4.77 のすぐ横)
            robot.transform.position = new Vector3(-2.00f, 0.00f, -4.55f);
            robot.transform.rotation = Quaternion.Euler(0f, 90f, 0f); // 赤ゾーン (+X方向) に正対

            // 1. Rigidbody & 自律コントローラ
            Rigidbody rb = robot.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

            robot.AddComponent<OpponentRobotController>();

            // マテリアル群の生成
            Material aluMat = CreateMat("Mat_Opp_Aluminum", new Color(0.76f, 0.78f, 0.82f, 1f), 0.75f, 0.65f);
            Material darkFrameMat = CreateMat("Mat_Opp_DarkFrame", new Color(0.18f, 0.20f, 0.23f, 1f), 0.5f, 0.5f);
            Material blueArmorMat = CreateMat("Mat_Opp_BlueArmor", new Color(0.10f, 0.35f, 0.88f, 1f), 0.3f, 0.65f);
            Material wheelRubberMat = CreateMat("Mat_Opp_Wheel", new Color(0.12f, 0.12f, 0.14f, 1f), 0.1f, 0.3f);
            Material airTankMat = CreateMat("Mat_Opp_AirTank", new Color(0.15f, 0.52f, 0.85f, 1f), 0.65f, 0.75f);
            Material yellowBaseMat = CreateMat("Mat_Opp_EStopYellow", new Color(0.95f, 0.80f, 0.05f, 1f), 0.1f, 0.6f);
            Material redBtnMat = CreateMat("Mat_Opp_EStopRed", new Color(0.92f, 0.10f, 0.10f, 1f), 0.1f, 0.6f);
            Material ledBlueMat = CreateEmissiveMat("Mat_Opp_LED", new Color(0.0f, 0.85f, 1.0f, 1f), new Color(0.0f, 1.5f, 2.0f, 1f));
            Material mascotSkinMat = CreateMat("Mat_Opp_MascotSkin", new Color(0.98f, 0.82f, 0.65f, 1f), 0.05f, 0.35f);
            Material mascotUniformMat = CreateMat("Mat_Opp_MascotBlue", new Color(0.12f, 0.42f, 0.95f, 1f), 0.1f, 0.45f);
            Material mascotWhiteMat = CreateMat("Mat_Opp_MascotWhite", new Color(0.95f, 0.95f, 0.97f, 1f), 0.1f, 0.45f);
            Material bucketPolyMat = CreateTransparentMat("Mat_Opp_PolyBucket", new Color(0.86f, 0.94f, 1.0f, 0.42f));

            // 2. オムニホイールベース車体 (880mm x 880mm x 260mm) - ルール3.2.2準拠
            BuildDetailedOmniChassis(robot.transform, aluMat, darkFrameMat, blueArmorMat, wheelRubberMat, airTankMat, yellowBaseMat, redBtnMat, ledBlueMat);

            // 3. 雑巾投擲シューター機構 (前方デッキ)
            BuildShooterMechanism(robot.transform, aluMat, darkFrameMat, wheelRubberMat);

            // 4. 教室用椅子 (新JIS 6号実寸CADメッシュ 24,880頂点) - ルール3.1.2 / 5.5準拠
            IsaacSceneMeta meta = FieldMapBuilder.LoadSceneMeta(out byte[] binData);
            BuildClassroomChair(robot.transform, meta, binData, aluMat);

            // 5. 多関節チームマスコット (360mm x 360mm x 660mm, 椅子に着席・固定ベルト付き) - ルール3.1.3準拠
            BuildDetailedMascot(robot.transform, mascotSkinMat, mascotUniformMat, mascotWhiteMat, darkFrameMat, redBtnMat);

            // 6. ツインアルミ支柱 & 移動バケツ (PO-24A実寸CADメッシュ, 上面高さ1.80m, 中空コライダー構造) - ルール3.2.3 / 5.3準拠
            BuildMovingBucketAssembly(robot.transform, meta, binData, aluMat, redBtnMat, bucketPolyMat);

            Selection.activeGameObject = robot;
            EditorUtility.SetDirty(robot);
            Debug.Log("<color=cyan>[OpponentRobotBuilder]</color> 2026公式ルール完全準拠・高精度相手ロボット (CAD椅子・CADバケツ・4輪オムニ・非常停止2基・LED・多関節マスコット・安全回廊ナビ搭載) を生成しました!");
        }

        private static void BuildDetailedOmniChassis(
            Transform root,
            Material aluMat,
            Material darkFrameMat,
            Material blueArmorMat,
            Material wheelMat,
            Material airTankMat,
            Material yellowMat,
            Material redMat,
            Material ledMat)
        {
            GameObject chassis = new GameObject("Chassis_OmniBase");
            chassis.transform.SetParent(root, false);

            // LiDAR反射用の車体コアBoxCollider (860mm x 260mm x 860mm)
            BoxCollider baseCol = chassis.AddComponent<BoxCollider>();
            baseCol.center = new Vector3(0f, 0.15f, 0f);
            baseCol.size = new Vector3(0.86f, 0.26f, 0.86f);

            // 下部デッキプレート & 上部デッキプレート
            CreateBox(chassis.transform, "Bottom_Deck_Plate", new Vector3(0f, 0.06f, 0f), new Vector3(0.80f, 0.015f, 0.80f), darkFrameMat, false);
            CreateBox(chassis.transform, "Upper_Deck_Plate", new Vector3(0f, 0.24f, 0f), new Vector3(0.82f, 0.015f, 0.82f), aluMat, false);

            // アルミフレーム外周ビーム (上下4本ずつ + 垂直柱4本)
            float halfW = 0.41f;
            for (int level = 0; level < 2; level++)
            {
                float y = (level == 0) ? 0.08f : 0.23f;
                CreateBox(chassis.transform, $"Frame_Beam_F_{level}", new Vector3(0f, y, halfW), new Vector3(0.84f, 0.03f, 0.03f), aluMat, false);
                CreateBox(chassis.transform, $"Frame_Beam_B_{level}", new Vector3(0f, y, -halfW), new Vector3(0.84f, 0.03f, 0.03f), aluMat, false);
                CreateBox(chassis.transform, $"Frame_Beam_L_{level}", new Vector3(-halfW, y, 0f), new Vector3(0.03f, 0.03f, 0.84f), aluMat, false);
                CreateBox(chassis.transform, $"Frame_Beam_R_{level}", new Vector3(halfW, y, 0f), new Vector3(0.03f, 0.03f, 0.84f), aluMat, false);
            }

            // 青チームサイドアーマーパネル & 360度視認用電源表示LEDストリップ (ルール3.2.8)
            CreateBox(chassis.transform, "Armor_Front", new Vector3(0f, 0.155f, 0.425f), new Vector3(0.72f, 0.13f, 0.01f), blueArmorMat, false);
            CreateBox(chassis.transform, "Armor_Back", new Vector3(0f, 0.155f, -0.425f), new Vector3(0.72f, 0.13f, 0.01f), blueArmorMat, false);
            CreateBox(chassis.transform, "Armor_Left", new Vector3(-0.425f, 0.155f, 0f), new Vector3(0.01f, 0.13f, 0.72f), blueArmorMat, false);
            CreateBox(chassis.transform, "Armor_Right", new Vector3(0.425f, 0.155f, 0f), new Vector3(0.01f, 0.13f, 0.72f), blueArmorMat, false);

            CreateBox(chassis.transform, "LED_Strip_Front", new Vector3(0f, 0.21f, 0.432f), new Vector3(0.68f, 0.015f, 0.008f), ledMat, false);
            CreateBox(chassis.transform, "LED_Strip_Back", new Vector3(0f, 0.21f, -0.432f), new Vector3(0.68f, 0.015f, 0.008f), ledMat, false);
            CreateBox(chassis.transform, "LED_Strip_Left", new Vector3(-0.432f, 0.21f, 0f), new Vector3(0.008f, 0.015f, 0.68f), ledMat, false);
            CreateBox(chassis.transform, "LED_Strip_Right", new Vector3(0.432f, 0.21f, 0f), new Vector3(0.008f, 0.015f, 0.68f), ledMat, false);

            // 4輪オムニホイール (45度配置、外周バレルローラー付き)
            Vector3[] wheelCorners = new Vector3[]
            {
                new Vector3(-0.36f, 0.075f,  0.36f),
                new Vector3( 0.36f, 0.075f,  0.36f),
                new Vector3(-0.36f, 0.075f, -0.36f),
                new Vector3( 0.36f, 0.075f, -0.36f)
            };
            float[] wheelYaws = new float[] { -45f, 45f, -135f, 135f };
            for (int i = 0; i < 4; i++)
            {
                GameObject wheelRoot = new GameObject($"OmniWheel_{i}");
                wheelRoot.transform.SetParent(chassis.transform, false);
                wheelRoot.transform.localPosition = wheelCorners[i];
                wheelRoot.transform.localRotation = Quaternion.Euler(0f, wheelYaws[i], 0f);

                // メインホイールハブ
                GameObject hub = CreateCylinder(wheelRoot.transform, "Wheel_Hub", Vector3.zero, new Vector3(0.15f, 0.022f, 0.15f), aluMat, false);
                hub.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);

                // 外周8個のオムニローラー
                for (int r = 0; r < 8; r++)
                {
                    float ang = r * 45f * Mathf.Deg2Rad;
                    Vector3 rPos = new Vector3(0f, Mathf.Sin(ang) * 0.065f, Mathf.Cos(ang) * 0.065f);
                    GameObject roller = CreateCylinder(wheelRoot.transform, $"Roller_{r}", rPos, new Vector3(0.028f, 0.018f, 0.028f), wheelMat, false);
                    roller.transform.localRotation = Quaternion.Euler(r * 45f, 0f, 0f);
                }
            }

            // 空圧タンク2基 (ルール3.2.7: 0.75MPa以下エアタンク) & 電装ボックス
            for (int t = 0; t < 2; t++)
            {
                float xOff = (t == 0) ? -0.28f : 0.28f;
                GameObject tank = CreateCylinder(chassis.transform, $"Pneumatic_AirTank_{t}", new Vector3(xOff, 0.31f, -0.22f), new Vector3(0.11f, 0.18f, 0.11f), airTankMat, false);
                tank.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
            }
            CreateBox(chassis.transform, "Electronics_ControlBox", new Vector3(0f, 0.30f, -0.32f), new Vector3(0.32f, 0.12f, 0.16f), darkFrameMat, false);

            // 非常停止スイッチ2箇所 (ルール3.2.8: 黄色い土台に取り付けた赤い押しボタン、最低2か所)
            Vector3[] estopPositions = new Vector3[]
            {
                new Vector3(-0.35f, 0.28f, -0.37f),
                new Vector3( 0.35f, 0.28f, -0.37f)
            };
            for (int e = 0; e < 2; e++)
            {
                GameObject eBase = CreateBox(chassis.transform, $"EStop_Base_{e}", estopPositions[e], new Vector3(0.075f, 0.055f, 0.075f), yellowMat, false);
                CreateCylinder(eBase.transform, "Red_Mushroom_Button", new Vector3(0f, 0.75f, 0f), new Vector3(0.65f, 0.35f, 0.65f), redMat, false);
            }
        }

        private static void BuildShooterMechanism(Transform root, Material aluMat, Material darkMat, Material rollerMat)
        {
            GameObject shooter = new GameObject("Cloth_Launcher_Mechanism");
            shooter.transform.SetParent(root, false);
            shooter.transform.localPosition = new Vector3(0f, 0.42f, 0.18f);
            shooter.transform.localRotation = Quaternion.Euler(-28f, 0f, 0f);

            // 左右サイドプレート & シュートガイド
            CreateBox(shooter.transform, "Shooter_Side_L", new Vector3(-0.14f, 0f, 0f), new Vector3(0.015f, 0.18f, 0.38f), aluMat, true);
            CreateBox(shooter.transform, "Shooter_Side_R", new Vector3( 0.14f, 0f, 0f), new Vector3(0.015f, 0.18f, 0.38f), aluMat, true);
            CreateBox(shooter.transform, "Shooter_Bed", new Vector3(0f, -0.06f, 0f), new Vector3(0.26f, 0.012f, 0.36f), darkMat, false);

            // 上下射出ローラー
            GameObject lowerRoller = CreateCylinder(shooter.transform, "Lower_Drive_Roller", new Vector3(0f, -0.02f, 0.12f), new Vector3(0.06f, 0.13f, 0.06f), rollerMat, false);
            lowerRoller.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
            GameObject upperRoller = CreateCylinder(shooter.transform, "Upper_Drive_Roller", new Vector3(0f, 0.05f, 0.12f), new Vector3(0.06f, 0.13f, 0.06f), rollerMat, false);
            upperRoller.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
        }

        private static void BuildClassroomChair(Transform root, IsaacSceneMeta meta, byte[] binData, Material fallbackMat)
        {
            GameObject chairObj = new GameObject("Classroom_Chair_JIS6");
            chairObj.transform.SetParent(root, false);
            // 車体上部デッキ (y=0.25m) の中央やや後方に配置
            chairObj.transform.localPosition = new Vector3(0f, 0.25f, -0.06f);
            chairObj.transform.localRotation = Quaternion.Euler(0f, 180f, 0f); // 背もたれを後方(-Z)に向ける

            bool loadedCad = false;
            if (meta != null && binData != null && meta.meshes != null)
            {
                foreach (var entry in meta.meshes)
                {
                    if (entry.name == "chair")
                    {
                        Mesh rawMesh = FieldMapBuilder.BuildUnityMesh(entry, binData, out Material[] mats);
                        EnsureAssetFolder();
                        for (int mIdx = 0; mIdx < mats.Length; mIdx++)
                        {
                            string matPath = $"{AssetFolder}/Mat_Opp_Chair_{mIdx}.mat";
                            AssetDatabase.DeleteAsset(matPath);
                            AssetDatabase.CreateAsset(mats[mIdx], matPath);
                        }
                        Mesh centeredMesh = RecenterMeshBottomCenter(rawMesh, "JIS6_Chair_CAD");
                        MeshFilter mf = chairObj.AddComponent<MeshFilter>();
                        mf.sharedMesh = centeredMesh;
                        MeshRenderer mr = chairObj.AddComponent<MeshRenderer>();
                        mr.sharedMaterials = mats;
                        loadedCad = true;
                        break;
                    }
                }
            }

            if (!loadedCad)
            {
                CreateBox(chairObj.transform, "Chair_Seat_Fallback", new Vector3(0f, 0.45f, 0f), new Vector3(0.36f, 0.03f, 0.40f), fallbackMat, false);
                CreateBox(chairObj.transform, "Chair_Back_Fallback", new Vector3(0f, 0.68f, 0.18f), new Vector3(0.36f, 0.22f, 0.03f), fallbackMat, false);
            }

            // LiDAR反射用の椅子コライダー (座面 + 背もたれ)
            BoxCollider seatCol = chairObj.AddComponent<BoxCollider>();
            seatCol.center = new Vector3(0f, 0.40f, 0f);
            seatCol.size = new Vector3(0.42f, 0.76f, 0.44f);
        }

        private static void BuildDetailedMascot(
            Transform root,
            Material skinMat,
            Material uniformMat,
            Material whiteMat,
            Material darkMat,
            Material accentMat)
        {
            // ルール3.1.3: 縦300mm×横300mm×高さ600mm以上。椅子に着席固定。
            // 椅子座面高さ = 0.25m (デッキ) + 0.46m (JIS6座面) = 0.71m
            GameObject mascot = new GameObject("Team_Mascot");
            mascot.transform.SetParent(root, false);
            mascot.transform.localPosition = new Vector3(0f, 0.71f, -0.05f);

            // LiDAR反射用マスコット全体コライダー (360mm x 660mm x 360mm)
            BoxCollider mascotCol = mascot.AddComponent<BoxCollider>();
            mascotCol.center = new Vector3(0f, 0.33f, 0.02f);
            mascotCol.size = new Vector3(0.36f, 0.66f, 0.36f);

            // 胴体 (青チームビブス着用)
            CreateBox(mascot.transform, "Mascot_Torso_Bib", new Vector3(0f, 0.24f, -0.02f), new Vector3(0.26f, 0.30f, 0.20f), uniformMat, false);
            CreateBox(mascot.transform, "Mascot_Chest_Number", new Vector3(0f, 0.25f, 0.085f), new Vector3(0.14f, 0.14f, 0.01f), whiteMat, false);

            // 椅子固定用シートベルト (ルール3.1.3: 競技中に落ちないよう固定)
            CreateBox(mascot.transform, "Safety_SeatBelt_Waist", new Vector3(0f, 0.14f, -0.04f), new Vector3(0.28f, 0.04f, 0.26f), darkMat, false);

            // 頭部 + チームヘルメット + 目・鼻・口
            GameObject head = CreateSphere(mascot.transform, "Mascot_Head", new Vector3(0f, 0.50f, -0.01f), new Vector3(0.22f, 0.22f, 0.22f), skinMat, false);
            CreateSphere(mascot.transform, "Mascot_Helmet", new Vector3(0f, 0.54f, -0.01f), new Vector3(0.24f, 0.16f, 0.24f), uniformMat, false);
            CreateBox(mascot.transform, "Mascot_Helmet_Visor", new Vector3(0f, 0.56f, 0.10f), new Vector3(0.18f, 0.02f, 0.08f), whiteMat, false);
            CreateSphere(head.transform, "Eye_L", new Vector3(-0.18f, 0.05f, 0.44f), new Vector3(0.10f, 0.10f, 0.05f), darkMat, false);
            CreateSphere(head.transform, "Eye_R", new Vector3( 0.18f, 0.05f, 0.44f), new Vector3(0.10f, 0.10f, 0.05f), darkMat, false);
            CreateSphere(head.transform, "Nose", new Vector3(0f, -0.02f, 0.48f), new Vector3(0.08f, 0.08f, 0.08f), accentMat, false);

            // 両腕 (360mm幅を満たすポーズ)
            for (int s = 0; s < 2; s++)
            {
                float dir = (s == 0) ? -1f : 1f;
                GameObject arm = CreateCylinder(mascot.transform, $"Mascot_Arm_{s}", new Vector3(dir * 0.16f, 0.28f, 0.03f), new Vector3(0.065f, 0.12f, 0.065f), uniformMat, false);
                arm.transform.localRotation = Quaternion.Euler(-25f, 0f, dir * -18f);
                CreateSphere(mascot.transform, $"Mascot_Hand_{s}", new Vector3(dir * 0.18f, 0.36f, 0.10f), new Vector3(0.075f, 0.075f, 0.075f), skinMat, false);
            }

            // 着席した両脚 (太もも + すね + 靴)
            for (int l = 0; l < 2; l++)
            {
                float xOff = (l == 0) ? -0.08f : 0.08f;
                CreateBox(mascot.transform, $"Mascot_Thigh_{l}", new Vector3(xOff, 0.05f, 0.08f), new Vector3(0.09f, 0.08f, 0.22f), whiteMat, false);
                CreateBox(mascot.transform, $"Mascot_Shin_{l}", new Vector3(xOff, -0.08f, 0.17f), new Vector3(0.08f, 0.18f, 0.08f), uniformMat, false);
                CreateBox(mascot.transform, $"Mascot_Shoe_{l}", new Vector3(xOff, -0.18f, 0.20f), new Vector3(0.09f, 0.06f, 0.14f), darkMat, false);
            }
        }

        private static void BuildMovingBucketAssembly(
            Transform root,
            IsaacSceneMeta meta,
            byte[] binData,
            Material aluMat,
            Material redLineMat,
            Material bucketPolyMat)
        {
            // ルール3.2.3:
            // a) バケツは開口部を上方向に向け、競技中は常に水平を維持できるよう固定。
            // b) バケツ上面は床面から 1200mm〜2100mm の範囲内 (1.80m に設定)。
            // c) ロボット・マスコットのいかなる部分もバケツの 2L 目盛りより上に出ない (マスコット頭頂部=1.37m < バケツ底面=1.545m)。
            const float BucketTopHeight = 1.80f;
            const float BucketHeight = 0.255f;   // エンテック PO-24A 規格 (H255mm)
            const float BucketRadius = 0.1365f;  // 外径 273mm / 2
            const float BucketBottomY = BucketTopHeight - BucketHeight; // 1.545m
            const float BucketPosZ = 0.24f;      // 車体前方寄り

            // ツインアルミ支柱タワー (車体デッキ y=0.25m からバケツ底面 y=1.545m まで)
            GameObject tower = new GameObject("Bucket_Support_Tower");
            tower.transform.SetParent(root, false);
            float towerHeight = BucketBottomY - 0.25f;
            float towerMidY = 0.25f + towerHeight * 0.5f;

            CreateBox(tower.transform, "Tower_Pillar_L", new Vector3(-0.08f, towerMidY, BucketPosZ), new Vector3(0.03f, towerHeight, 0.03f), aluMat, true);
            CreateBox(tower.transform, "Tower_Pillar_R", new Vector3( 0.08f, towerMidY, BucketPosZ), new Vector3(0.03f, towerHeight, 0.03f), aluMat, true);
            CreateBox(tower.transform, "Tower_CrossBrace_Low", new Vector3(0f, 0.75f, BucketPosZ), new Vector3(0.18f, 0.025f, 0.025f), aluMat, false);
            CreateBox(tower.transform, "Tower_CrossBrace_High", new Vector3(0f, 1.25f, BucketPosZ), new Vector3(0.18f, 0.025f, 0.025f), aluMat, false);
            CreateCylinder(tower.transform, "Bucket_Base_MountPlate", new Vector3(0f, BucketBottomY, BucketPosZ), new Vector3(0.22f, 0.01f, 0.22f), aluMat, false);

            // 移動バケツ本体 (PO-24A 実寸CADメッシュ)
            GameObject bucketObj = new GameObject("Opponent_Moving_Bucket");
            bucketObj.transform.SetParent(root, false);
            // GroundTruthPublisher や GoalDetector が上面中心を基準にできるよう、バケツ中心 (y = 1.6725m) に配置
            bucketObj.transform.localPosition = new Vector3(0f, BucketBottomY + BucketHeight * 0.5f, BucketPosZ);

            bool loadedBucketCad = false;
            if (meta != null && binData != null && meta.meshes != null)
            {
                foreach (var entry in meta.meshes)
                {
                    if (entry.name == "bucket1_blue")
                    {
                        Mesh rawBucket = FieldMapBuilder.BuildUnityMesh(entry, binData, out _);
                        Mesh centeredBucket = RecenterMeshCenter(rawBucket, "PO24A_Bucket_CAD");
                        MeshFilter mf = bucketObj.AddComponent<MeshFilter>();
                        mf.sharedMesh = centeredBucket;
                        MeshRenderer mr = bucketObj.AddComponent<MeshRenderer>();
                        mr.sharedMaterial = bucketPolyMat;
                        loadedBucketCad = true;
                        break;
                    }
                }
            }

            if (!loadedBucketCad)
            {
                GameObject cyl = CreateCylinder(bucketObj.transform, "Bucket_Visual_Fallback", Vector3.zero, new Vector3(BucketRadius * 2f, BucketHeight * 0.5f, BucketRadius * 2f), bucketPolyMat, false);
            }

            // 2L目盛りリング表示 (底面から50mm位置 = local y = -0.1275 + 0.05 = -0.0775m)
            CreateCylinder(bucketObj.transform, "Bucket_2L_LimitLine", new Vector3(0f, -0.0775f, 0f), new Vector3(0.252f, 0.004f, 0.252f), redLineMat, false);

            // 中空バケツコライダー構造 (底面コライダー + 8枚の側壁コライダーリング: 外からはLiDARが当たり、上からは雑巾が入る)
            BoxCollider bottomCol = bucketObj.AddComponent<BoxCollider>();
            bottomCol.center = new Vector3(0f, -BucketHeight * 0.45f, 0f);
            bottomCol.size = new Vector3(BucketRadius * 1.8f, 0.025f, BucketRadius * 1.8f);

            for (int w = 0; w < 8; w++)
            {
                float angleDeg = w * 45f;
                float rad = angleDeg * Mathf.Deg2Rad;
                GameObject wallSeg = new GameObject($"Bucket_WallCollider_{w}");
                wallSeg.transform.SetParent(bucketObj.transform, false);
                wallSeg.transform.localPosition = new Vector3(Mathf.Sin(rad) * (BucketRadius * 0.92f), 0f, Mathf.Cos(rad) * (BucketRadius * 0.92f));
                wallSeg.transform.localRotation = Quaternion.Euler(0f, angleDeg, 0f);
                BoxCollider wc = wallSeg.AddComponent<BoxCollider>();
                wc.size = new Vector3(0.11f, BucketHeight, 0.02f);
            }

            // 真値TFパブリッシャー
            bucketObj.AddComponent<OpponentGroundTruthPublisher>();

            // バケツ開口部ゴール判定トリガー (上面中心 local y = +0.1275m)
            GameObject goalTriggerObj = new GameObject("Goal_Trigger");
            goalTriggerObj.transform.SetParent(bucketObj.transform, false);
            goalTriggerObj.transform.localPosition = new Vector3(0f, BucketHeight * 0.5f, 0f);
            SphereCollider sc = goalTriggerObj.AddComponent<SphereCollider>();
            sc.isTrigger = true;
            sc.radius = BucketRadius * 0.95f;

            goalTriggerObj.AddComponent<OpponentBucketGoalDetector>();
        }

        private const string AssetFolder = "Assets/Resources/Opponent";

        private static void EnsureAssetFolder()
        {
            if (!AssetDatabase.IsValidFolder("Assets/Resources"))
            {
                AssetDatabase.CreateFolder("Assets", "Resources");
            }
            if (!AssetDatabase.IsValidFolder(AssetFolder))
            {
                AssetDatabase.CreateFolder("Assets/Resources", "Opponent");
            }
        }

        private static Mesh RecenterMeshBottomCenter(Mesh src, string newName)
        {
            EnsureAssetFolder();
            string assetPath = $"{AssetFolder}/{newName}.asset";
            Mesh m = UnityEngine.Object.Instantiate(src);
            m.name = newName;
            Bounds b = m.bounds;
            Vector3 offset = new Vector3(b.center.x, b.min.y, b.center.z);
            Vector3[] verts = m.vertices;
            for (int i = 0; i < verts.Length; i++)
            {
                verts[i] -= offset;
            }
            m.vertices = verts;
            m.RecalculateBounds();
            AssetDatabase.DeleteAsset(assetPath);
            AssetDatabase.CreateAsset(m, assetPath);
            return m;
        }

        private static Mesh RecenterMeshCenter(Mesh src, string newName)
        {
            EnsureAssetFolder();
            string assetPath = $"{AssetFolder}/{newName}.asset";
            Mesh m = UnityEngine.Object.Instantiate(src);
            m.name = newName;
            Bounds b = m.bounds;
            Vector3 offset = b.center;
            Vector3[] verts = m.vertices;
            for (int i = 0; i < verts.Length; i++)
            {
                verts[i] -= offset;
            }
            m.vertices = verts;
            m.RecalculateBounds();
            AssetDatabase.DeleteAsset(assetPath);
            AssetDatabase.CreateAsset(m, assetPath);
            return m;
        }

        private static GameObject CreateBox(Transform parent, string name, Vector3 localPos, Vector3 scale, Material mat, bool keepCollider)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            if (!keepCollider)
            {
                UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());
            }
            return go;
        }

        private static GameObject CreateCylinder(Transform parent, string name, Vector3 localPos, Vector3 scale, Material mat, bool keepCollider)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            if (!keepCollider)
            {
                UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());
            }
            return go;
        }

        private static GameObject CreateSphere(Transform parent, string name, Vector3 localPos, Vector3 scale, Material mat, bool keepCollider)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            if (!keepCollider)
            {
                UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());
            }
            return go;
        }

        private static Material CreateMat(string name, Color color, float metallic, float smoothness)
        {
            EnsureAssetFolder();
            string assetPath = $"{AssetFolder}/{name}.mat";
            Shader shader = Shader.Find("Standard") ?? Shader.Find("Universal Render Pipeline/Lit");
            Material mat = new Material(shader) { name = name, color = color };
            if (mat.HasProperty("_Metallic")) mat.SetFloat("_Metallic", metallic);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);
            AssetDatabase.DeleteAsset(assetPath);
            AssetDatabase.CreateAsset(mat, assetPath);
            return mat;
        }

        private static Material CreateEmissiveMat(string name, Color baseColor, Color emissionColor)
        {
            Material mat = CreateMat(name, baseColor, 0.2f, 0.8f);
            mat.EnableKeyword("_EMISSION");
            if (mat.HasProperty("_EmissionColor")) mat.SetColor("_EmissionColor", emissionColor);
            EditorUtility.SetDirty(mat);
            return mat;
        }

        private static Material CreateTransparentMat(string name, Color color)
        {
            EnsureAssetFolder();
            string assetPath = $"{AssetFolder}/{name}.mat";
            Shader shader = Shader.Find("Standard") ?? Shader.Find("Universal Render Pipeline/Lit");
            Material mat = new Material(shader) { name = name, color = color };
            if (mat.HasProperty("_Mode"))
            {
                mat.SetFloat("_Mode", 3f);
                mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                mat.SetInt("_ZWrite", 0);
                mat.DisableKeyword("_ALPHATEST_ON");
                mat.EnableKeyword("_ALPHABLEND_ON");
                mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                mat.renderQueue = 3000;
            }
            AssetDatabase.DeleteAsset(assetPath);
            AssetDatabase.CreateAsset(mat, assetPath);
            return mat;
        }
    }
}
