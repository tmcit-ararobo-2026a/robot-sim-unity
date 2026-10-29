# robot-sim-unity (NHKロボコン2026 Unity ロボットシミュレータ)

都立産技高専 荒川ロボコン 2026 Aチーム（GN10）向けのロボット・フィールド統合シミュレータです。  
Isaac Sim と同一のフィールド・機体ジオメトリ、Livox Mid-360 80万点非反復走査パターン、RealSense D435 深度カメラ、および布射出弾道物理を Unity 6 上で高効率に再現し、実機・Isaac Sim と全く同一の ROS 2 Humble ノード群（`gn10_pointcloud_localization`, `gn10_navigation`, `realsense_dust_cloth`）をそのまま連携動作させることができます。

---

## 1. 特徴

- **Isaac Sim と同一の 3D フィールド・機体モデル**:
  - `main.usd` および `v3_3.usd` のメッシュ・コライダー・マテリアルを移植し、`nhk2026_map.json` のミリ単位定義（外壁、中央教壇、バケツ台座、机、椅子、旗支柱）と合致。
- **フィールド物体の物理挙動**:
  - 机・椅子・バケツ・旗・バケツ台座の22物体は動的 Rigidbody として、ロボットの衝突で移動・回転・転倒します。床・外壁・中央教壇は固定です。
  - 雑巾はルールブック5.1に基づき約0.048kg、バケツ本体は5.3に基づき約0.978kgです。固定バケツの錘は質量未公表のため未モデル化で、`FieldMapBuilder` の `Fixed Bucket Ballast Mass` に実測した追加質量を指定できます（初期値0kg）。机8kg・椅子5kg・旗10kg・バケツ台座3kgは引き続き仮設定です。`Movable Field Props` で調整でき、`Simulate Loose Props` で可動化を切り替えられます。
  - 摩擦・反発は乾燥・清浄な接触面を想定した承認済み推定値です。`Assets/Resources/Physics/SurfaceProfiles` の Physics Material を各コライダーに割り当てています。摩擦の合成は `Average`、反発の合成は `Minimum` です。机・椅子の脚先と旗の布・支柱・土台は材質を分けています。
  - `RobotSim > Apply Approved Field Physics and Save` でフィールド・相手機の設定を適用できます。自機 `Robot` の質量・摩擦・反発には触れません。相手機の移動バケツと椅子は車体に固定されており、個別の動的 Rigidbody を追加しません。スーパー雑巾の専用Prefabは現在ありません。
  - 見た目のCADメッシュを維持し、動的物体の当たり判定は複合プリミティブで近似します。机・椅子は部材単位、バケツは開いた上面を残した側壁・底、旗は台座・支柱・旗部分に分けます。
  - 床の色分け・ラインに含まれる厚さ1mmの縁を衝突形状から除き、平らな床コライダーで支えます。外壁・中央教壇の形状は維持します。
  - バケツ台座と中央教壇は実寸の閉じた箱形コライダーで支えます。台座とバケツは独立した物理オブジェクトで、台座が倒れたり移動したりするとバケツも接触・摩擦に応じて動きます。旧メッシュの裏向き面と、斜め衝突時に三角形の縁で機体が持ち上がる挙動を避けています。
- **Livox Mid-360 LiDAR / IMU の忠実な再現**:
  - Isaac Sim 拡張機能の `mid360.npy` から抽出した 80 万本の非反復走査角度バイナリ (`mid360_pattern.bin`) を直接読み込み、各点の時刻で位置・姿勢を補間し、表示メッシュの三角形をCompute Shader＋2段BVHで直接測距。
  - 実機・Isaac Sim の逆さチルトマウント姿勢（`x=0.24, y=-0.25, z=1.09, roll=3.13 rad, pitch=-0.273 rad`）で照射。
  - 床・自機・無反射のサンプルを間引かず配信。各点は計測時のLiDAR座標系に保持し、走査歪みの補正は受信側で行います。
- **RealSense D435 & 布射出・3D 弾道追跡**:
  - 実機カメラの内部パラメータ・画角（30 Hz, `rgb8` カラー + `16UC1` 16-bit 深度 mm単位）を配信。
  - `/belt/throw` 射出信号により、`realsense_dust_cloth` の CUDA 追跡ゲートを正確に通過する放物線弾道物理を再現。
  - 発射後の布は重力・空気抵抗と連続衝突判定を持つ動的 Rigidbody で動き、机・壁・バケツなどに接触します。深度重畳と得点判定には、その物理位置を渡します。
  - 発射する布は `Assets/Prefab/cloth.prefab` をインスタンス化します。外見と物理設定はこのプレハブで編集でき、シーンに常設の `cloth_pool` は置きません。
  - 布の柔らかさを近似して、衝突判定は 12 × 8 × 12 cm の箱に縮めています。見た目のメッシュと質量は維持し、バケツの開口を通れる寸法にしています。
- **Rigidbody による自機の物理走行 & 手動キーボード操作**:
  - 実機CADの下部10cmを単一の `Chassis ground contact` Box Collider にまとめ、床との接触面を平らにしています。上部はメッシュに沿った複数のBox Colliderを維持します。慣性軸も機体の軸に合わせ、ロール・ピッチ固定時の微小な姿勢ずれを抑えます。
  - `/robot/operation/cmd_vel` および `/cmd_vel` を目標速度として、`FixedUpdate` で駆動力・旋回トルクを加えます。最高速度は 2 m/s、最高角速度は 3.14 rad/s です。
  - 重力と連続衝突判定を有効にし、壁との接触・他の Rigidbody の押し出しを物理演算で処理します。非接地時は駆動力を加えません。
  - 自機の重心の水平位置を `Robot` / `base_link` の原点に合わせます。非対称なCAD形状による回転中心のずれを防ぎ、並進と旋回を同時に指令しても機体原点が重心の周囲を振れ回らないようにします。重心の高さは維持します。
  - `W/A/S/D`（前後左右移動）・`Q/E`（旋回）で手動操作でき、キー解放・0.25秒の通信途絶時は物理的に制動します。
  - キーで並進を始めたときの機体の向きを移動の基準として保持します。`W`などを押したまま`Q/E`で旋回しても進行方向は曲がりません。並進キーをすべて離して押し直すと、その時点の向きを新しい基準にします。ROSの速度指令は機体座標のままです。
  - 速度指令には加減速のランプを設け、並進4m/s²・制動6m/s²、旋回8rad/s²・旋回制動12rad/s²で変化させます。指令上は最高速まで約0.5秒、停止まで約0.33秒です。旋回の減速・切り返しでは強めの制動を使います。物体を押す駆動力の上限は別に維持し、`Robot` の走行コンポーネントから調整できます。
  - 車体単位のオムニ駆動近似です。車輪ごとの回転・モータートルク・タイヤのスリップは未モデル化で、横転防止のためロール・ピッチを固定しています。相手機は従来の Kinematic 制御です。
- **バックグラウンド実行対応**:
  - Unity が背面ウィンドウにある状態でも実行を継続します。`/clock` は Unity の物理ステップに同期し、ポーズ・`timeScale` に追従します。IMU と布射出・深度重畳は実際の Rigidbody の状態を参照します。

物理接触の推定値（静止摩擦 / 動摩擦 / 反発）:

| 接触面 | 静止摩擦 | 動摩擦 | 反発 |
| --- | ---: | ---: | ---: |
| 床・床上のシート類 | 0.60 | 0.45 | 0.05 |
| 雑巾 | 0.75 | 0.55 | 0.00 |
| 机天板・椅子座面・相手機外装 | 0.45 | 0.30 | 0.05 |
| 机・椅子の脚先（樹脂キャップを仮定） | 0.55 | 0.40 | 0.00 |
| 固定・移動バケツ | 0.40 | 0.30 | 0.10 |
| 木製台・教壇・外周フェンス | 0.50 | 0.35 | 0.05 |
| 旗の布 | 0.60 | 0.45 | 0.00 |
| 旗の支柱 | 0.35 | 0.25 | 0.05 |
| 旗の土台 | 0.55 | 0.40 | 0.00 |

上表は資料記載の測定値ではありません。雑巾は剛体近似なので、これらの係数だけでは布の折れ曲がりや旗への掛かり方を再現できません。

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
| `/livox/imu` | `sensor_msgs/msg/Imu` | 200 Hz（既定） | Mid-360 センサ座標系角速度・加速度 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | RealSense カラー映像 (`rgb8`, 640x480) |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | RealSense 16-bit 深度画像 (`16UC1`, mm単位) |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | カメラ内部パラメータ |

`/livox/lidar` は公式 `livox_ros_driver2` の `xfer_format=0`（PointXYZRTLT）に合わせた、パディングなし・リトルエンディアンの26バイト／点です。

| フィールド | 型 | バイト位置 |
| --- | --- | ---: |
| `x`, `y`, `z` | FLOAT32 | 0, 4, 8 |
| `intensity` | FLOAT32 | 12 |
| `tag` | UINT8 | 16 |
| `line` | UINT8 | 17 |
| `timestamp` | FLOAT64 | 18 |

座標はm、反射強度は0〜255（未校正は0）、`tag` は理想的な通常反射として0です。`line` は走査順に0〜3を繰り返し、メッセージ境界でも位相を維持します。`header.stamp` は先頭点のシミュレーション時刻（秒・ナノ秒）、各点の `timestamp` は同じ時計の絶対ナノ秒値です。受信側でバイト位置を固定していた場合は旧25バイト形式からの変更が必要です。

既定の `Simulate Motion Distortion` では、物理ステップごとに機体・表示メッシュの剛体姿勢を記録し、各点の計測時刻に位置を線形補間、回転を球面補間して個別に測距します。センサー取付位置も機体姿勢で回転させます。毎秒20万サンプルの取得を物理ステップで進め、10 Hzなら2万点ずつ配信します。レンダーフレームが遅れても、過去の点を最新姿勢で再計測しません。生点群に走査歪みが入り、deskewは受信側が行います。

このモードの既定（`Use Gpu` 有効）は、`MeshFilter` の表示用三角形と `SkinnedMeshRenderer.BakeMesh` の頂点をCompute Shaderで直接計測します。メッシュ内のBVHと、物体の移動範囲を含むシーンBVHの2段で探索し、Colliderのない薄い部品や穴のある形状も扱います。剛体の位置・回転・スケール、スキンメッシュの頂点を各点の時刻で補間します。静的メッシュは共有・キャッシュし、スキン形状だけ更新します。非同期GPU readbackは取得順に処理し、読み戻し失敗時も保存済みのメッシュ・姿勢でCPUの三角形BVHを再実行します。`Use Gpu` 無効時やGPU非対応時にも同じメッシュ計測を使い、Colliderへ置き換えません。キューが64区間に達した場合だけ古いGPU結果を待ち、サンプルを捨てずにメモリを制限します。

メッシュのRead/Writeが必要です。読み取れないメッシュは明示的なエラーにします。静的な `MeshFilter.sharedMesh` の頂点を直接変更する場合は、新しいMeshへの差し替えが必要です。現在の計測対象は有効なMeshRenderer／SkinnedMeshRendererの三角形で、両面を検出します。テクスチャの透過・alpha clip、頂点シェーダーによる見た目の変形、Terrain／パーティクル／LineRenderer、親の非一様スケールと回転によるシアー変形は再現しません。SkinnedMeshRendererのスキン変形は頂点補間しますが、トポロジーの変更は区間内で補間しません。物理ステップ内の衝突軌道や物体の出現・消滅時刻も補間による近似です。

時刻間隔は20万サンプル/秒の理想化モデルで、実機UDPのパケット単位の時刻ゆらぎは再現しません。`Simulate Motion Distortion` を無効にした場合だけ従来の深度方式（GPUまたはCollider）で瞬間のシーンをまとめて計測します。

IMUは点群と同じ物理ステップの時計・Rigidbody状態から取得します。既定200 Hzの各サンプル時刻（5 ms刻み）で姿勢・角速度を補間し、区間の速度差から求めた加速度、重力、IMUチップ位置の回転加速度を使って値を確定します。Rigidbody速度の基準である重心からIMUチップまでの距離を使います。配信スレッドは保存済みメッセージを送るだけで、後の状態を過去時刻へ割り当てません。100 msの配信バッファは計測タイムスタンプを変更しません。`header.stamp` は公式ドライバと同様に取得時刻を秒・ナノ秒に分けた値で、シミュレーションではPTP/GPS時計の代わりに `/clock` と共通のシミュレーション時計を使います。IMU・点群ヘッダー・`/clock` のナノ秒変換は共通化しています。

物理ステップは10 msなので、その間のIMUは補間による近似です。物理ステップ内の衝撃や実機のノイズ・遅延・時計ゆらぎは未モデル化です。角速度はrad/s、加速度は公式UDPと `livox_ros_driver2` に合わせてg単位で配信します。物理計算はm/s²で行い、ROSメッセージ生成時だけ標準重力 `1 g = 9.80665 m/s²` で割ります。この係数はシーンの重力設定とは独立です。Unityの既定重力9.81 m/s²なら静止時の加速度ノルムは約1.00034 gです。旧シミュレータのm/s²出力を前提とした受信側は単位の確認が必要です。SI単位へ戻す場合は9.80665を掛けてください。[公式IMU配信処理](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)、[公式IMUプロトコル](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)。`RobotSim > Verify MID360 IMU Timing` で取得時刻・補間・Rigidbody連携・点群時計との一致を検査し、`Temp/Mid360ImuVerification.json` に記録します。

CustomMsg形式には対応していません。`RobotSim > Verify MID360 Scene Capture` の検証には、並進・旋回・可動物体の解析解との距離比較と、物理ステップをまたぐ2万点の取得・タイムスタンプ検査も含まれます。

`RobotSim > Verify MID360 Mesh BVH` でGPUメッシュ測距を検証できます。2万点の並進・旋回の解析解、複数の非同期取得、Colliderのない薄い形状、穴、非一様・負スケール、無効Renderer、スキン頂点の補間を検査し、結果は `Temp/Mid360MeshVerification.json` に保存します。

参考測定（この環境のMesa Intel RPL-P、Editor、約116万三角形・119物体）: 初回BVH構築は約7.1秒、実走査パターン2万点のGPU投入〜読み戻しは約101 msでした。性能保証値ではなく、描画や他センサーとの併用で実時間の配信Hzは低下し得ます。毎秒20万点はシミュレーション時間に対する取得レートです。

形式の参照元: [公式フィールド定義・配信処理](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)、[公式構造体・MID-360のline数](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/comm.h)。

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
- Play 中に `RobotSim > Verify Fired Cloth Collisions (Play Mode)` を実行すると、実際の発射処理を使い、20m/s での薄い壁・動的物体・フィールドの机との衝突、2種類のバケツへの4姿勢での進入、およびセンサー位置との一致を検証します。
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
