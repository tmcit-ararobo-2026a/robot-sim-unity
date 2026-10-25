# robot-sim-unity (NHKロボコン2026 Unity ロボットシミュレータ)

都立産技高専 荒川ロボコン 2026 Aチーム（GN10）向けのロボット・フィールド統合シミュレータです。  
Isaac Sim と同一のフィールド・機体ジオメトリ、Livox Mid-360 80万点非反復走査パターン、RealSense D435 深度カメラ、および布射出弾道物理を Unity 6 上で高効率に再現し、実機・Isaac Sim と全く同一の ROS 2 Humble ノード群（`gn10_pointcloud_localization`, `gn10_navigation`, `realsense_dust_cloth`）をそのまま連携動作させることができます。

---

## 1. 特徴

- **Isaac Sim と同一の 3D フィールド・機体モデル**:
  - `main.usd` および `v3_3.usd` のメッシュ・コライダー・マテリアルを移植し、`nhk2026_map.json` のミリ単位定義（外壁、中央教壇、バケツ台座、机、椅子、旗支柱）と合致。
- **フィールド物体の物理挙動**:
  - 机・椅子・バケツ・旗・バケツ台座の22物体は動的 Rigidbody として、ロボットの衝突で移動・回転・転倒します。床・外壁・中央教壇は固定です。
  - 質量は仮設定（机8kg・椅子5kg・バケツ0.6kg・旗10kg・バケツ台座3kg）です。`Field` の `FieldMapBuilder` の `Movable Field Props` で調整でき、`Simulate Loose Props` で可動化を切り替えられます。
  - 見た目のCADメッシュを維持し、動的物体の当たり判定は複合プリミティブで近似します。机・椅子は部材単位、バケツは開いた上面を残した側壁・底、旗は台座・支柱・旗部分に分けます。
  - 床の色分け・ラインに含まれる厚さ1mmの縁を衝突形状から除き、平らな床コライダーで支えます。外壁・中央教壇の形状は維持します。
  - バケツ台座と中央教壇は実寸の閉じた箱形コライダーで支えます。台座とバケツは独立した物理オブジェクトで、台座が倒れたり移動したりするとバケツも接触・摩擦に応じて動きます。旧メッシュの裏向き面と、斜め衝突時に三角形の縁で機体が持ち上がる挙動を避けています。
- **Livox Mid-360 LiDAR / IMU の忠実な再現**:
  - Isaac Sim 拡張機能の `mid360.npy` から抽出した 80 万本の非反復走査角度バイナリ (`mid360_pattern.bin`) を直接読み込み、マルチスレッド並列レイキャストを実行。
  - 実機・Isaac Sim の逆さチルトマウント姿勢（`x=0.24, y=-0.25, z=1.09, roll=3.13 rad, pitch=-0.273 rad`）で照射。
  - Linux UDP ソケット受信バッファ溢れ（パケットドロップ）を防ぐ床面サブサンプリングを実装。
- **RealSense D435 & 布射出・3D 弾道追跡**:
  - 実機カメラの内部パラメータ・画角（30 Hz, `rgb8` カラー + `16UC1` 16-bit 深度 mm単位）を配信。
  - `/belt/throw` 射出信号により、`realsense_dust_cloth` の CUDA 追跡ゲートを正確に通過する放物線弾道物理を再現。
- **Rigidbody による自機の物理走行 & 手動キーボード操作**:
  - `/robot/operation/cmd_vel` および `/cmd_vel` を目標速度として、`FixedUpdate` で駆動力・旋回トルクを加えます。最高速度は 2 m/s、最高角速度は 3.14 rad/s です。
  - 重力と連続衝突判定を有効にし、壁との接触・他の Rigidbody の押し出しを物理演算で処理します。非接地時は駆動力を加えません。
  - `W/A/S/D`（前後左右移動）・`Q/E`（旋回）で手動操作でき、キー解放・0.25秒の通信途絶時は物理的に制動します。
  - 速度指令には加減速のランプを設け、並進4m/s²・制動6m/s²、旋回8rad/s²・旋回制動12rad/s²で変化させます。指令上は最高速まで約0.5秒、停止まで約0.33秒です。旋回の減速・切り返しでは強めの制動を使います。物体を押す駆動力の上限は別に維持し、`Robot` の走行コンポーネントから調整できます。
  - 車体単位のオムニ駆動近似です。車輪ごとの回転・モータートルク・タイヤのスリップは未モデル化で、横転防止のためロール・ピッチを固定しています。相手機は従来の Kinematic 制御です。
- **バックグラウンド実行対応**:
  - Unity が背面ウィンドウにある状態でも実行を継続します。`/clock` は Unity の物理ステップに同期し、ポーズ・`timeScale` に追従します。IMU と布射出・深度重畳は実際の Rigidbody の状態を参照します。

---

## 2. 動作環境

- **OS**: Ubuntu 22.04 LTS (x86_64)
- **ROS 2**: Humble Hawksbill (Fast DDS: `rmw_fastrtps_cpp`)
- **Unity**: Unity 6 (`6000.0.85f1` 以降推奨)
- **Git LFS**: 大容量バイナリ (`.usd`, `.bin`) の管理に必要

---

## 3. セットアップ

### リポジトリのクローン (Git LFS 必須)
```bash
git lfs install
git clone https://github.com/tmcit-ararobo-2026a/robot-sim-unity.git
cd robot-sim-unity
git lfs pull
```

### Unity で開く
1. Unity Hub を起動し、「Add project from disk」からクローンした `robot-sim-unity` フォルダを選択して開きます。
2. Project ウィンドウから `Assets/Scenes/NHK2026_SimScene.unity` をダブルクリックして開きます。
3. エディタ上部の **Play ボタン (▶)** を押してシミュレーションを開始します。

---

## 4. 通信トピック仕様

### 配信トピック (Unity -> ROS 2)

| トピック名 | 型 | 周波数 | 概要 |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 100 Hz | シミュレーション同期時計 |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 10 Hz | 3D点群 (`livox_frame`, タイムスタンプ付) |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 100 Hz | Mid-360 センサ座標系角速度・加速度 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | RealSense カラー映像 (`rgb8`, 640x480) |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | RealSense 16-bit 深度画像 (`16UC1`, mm単位) |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | カメラ内部パラメータ |

### 購読トピック (ROS 2 -> Unity)

| トピック名 | 型 | 概要 |
| --- | --- | --- |
| `/robot/operation/cmd_vel` | `geometry_msgs/msg/Twist` | ナビゲーション自律走行速度指示 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 手動/汎用速度指示 |
| `/belt/throw` | `std_msgs/msg/Bool` | 布射出トリガー (`true` で射出) |
| `/belt/speed_ratio` | `std_msgs/msg/Float32` | 射出初速比率 (0.0〜1.5) |
| `/launcher/release_twist` | `geometry_msgs/msg/Twist` | ランチャー射出速度 |
| `/reset_cloth` | `std_msgs/msg/Bool` | 布位置リセット |

---

## 5. ROS 2 連携・動作確認

※ 実機用の各 ROS 2 ノードは**コード変更不要**でそのまま動作します。シミュレータ使用時は基本的に `use_sim_time:=true` を指定するだけです。

### 5.1 一括起動（自己位置推定 + ナビゲーション + RealSense + 統合RViz2）
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup unity_sim.launch.py use_sim_time:=true
```

---

### 5.2 個別起動する場合

#### 1. 自己位置推定 (`gn10_pointcloud_localization`)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch gn10_pointcloud_localization localization.launch.py use_sim_time:=true
```
- 初期起動時に `[GlobalSearch]` が走り、スタート座標 `(-4.15, -2.30, 1.57 rad)` に瞬時にロックされます。

#### 2. ナビゲーション (`gn10_navigation`)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch gn10_navigation navigation.launch.py use_sim_time:=true use_rviz:=true \
  map_file_path:=$(ros2 pkg prefix --share gn10_navigation)/config/nhk2026_map.json
# （またはソースツリーから直接指定する場合: map_file_path:=$HOME/ros2_ws/src/gn10-navigation/config/nhk2026_map.json）
```

#### 3. RealSense 布追跡 (`realsense_dust_cloth`)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_sim_time:=true use_direct_realsense:=false
```

---

### 5.3 動作確認コマンド

#### A. 手動移動の確認
- Unity の Game 画面で `W/A/S/D`（移動）・`Q/E`（旋回）を押している間ロボットが走行し、キーを離すと減速して停止します。
- 壁に向かって移動し、壁で機体とセンサー位置が止まることを確認できます。
- Play 中に `RobotSim > Verify Rigidbody Drive (Play Mode)` を実行すると、独立した物理シーンで落下・接地・前後左右移動・旋回・制動・指令タイムアウト・薄い壁との衝突・物体の押し出しを検証します。結果は `Temp/OmniDrivePhysicsVerification.json` に出力します。
- 同じ検証で実際のフィールドの机・椅子・バケツ・旗のCAD形状を使い、ロボットが押して動かせることと床が固定されていることも確認します。
- 4か所の可動台座上へのバケツ落下・台座の押し出しと、最高速度で中央段差へ正面・斜め・横移動を含む6方向から進入しても乗り越えないことを検証します。
- RViz2 上で `map -> base_link` TF および点群がマップに追従することを確認できます。

#### B. 自律移動（ナビゲーション）の確認
```bash
# 移動許可 (Move=2) を送信
ros2 topic pub --once /robot/navigation_command std_msgs/msg/UInt8 "{data: 2}"

# 目的座標を送信 (例: x=-2.0, y=-2.3)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -2.0, y: -2.3, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}"
```
- ※ RViz2 上部の **`2D Goal Pose`** ツールでマップ上の任意の場所をクリック＆ドラッグして移動させることも可能です。

#### C. RealSense 布射出・3D 追跡の確認
```bash
ros2 topic pub --once /belt/throw std_msgs/msg/Bool "{data: true}"
```
- ロボットから布が射出され、`realsense_dust_cloth` の CUDA 追跡ノードによって放物線軌跡（黄色いライン）および着地判定（`CLOTH LANDED`）が RViz2 に描画されます。

---

## 6. ディレクトリ構成

```
robot_sim_unity/
├── Assets/
│   ├── Models/            # 3Dモデル・マテリアル
│   ├── Ros2ForUnity/      # Unity ROS 2 バインディング (Fast DDS)
│   ├── Scenes/
│   │   └── NHK2026_SimScene.unity   # メインシミュレーションシーン
│   ├── Scripts/
│   │   ├── Control/       # KinematicOmniDrive.cs (Rigidbody 物理走行・手動操作、既存シーン互換名)
│   │   ├── Editor/        # フィールド自動構築エディタ拡張
│   │   ├── Sensors/       # LivoxMid360Sim.cs, RealSenseDepthSim.cs
│   │   └── Shooter/       # SimpleClothShooter.cs (布弾道物理)
│   ├── Shaders/           # 深度エンコードシェーダ
│   └── StreamingAssets/   # mid360_pattern.bin, isaac_meshes.bin
├── Packages/              # Unity パッケージ設定
└── ProjectSettings/       # プロジェクト設定
```

---

## 7. ライセンス

MIT License
