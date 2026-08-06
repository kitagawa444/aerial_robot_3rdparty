# mujoco_ros_control

ROS (catkin) パッケージ。URDF/xacro で記述されたロボットモデルを MuJoCo シミュレーション用の XML に自動変換し、`ros_control` インターフェースで制御するためのパッケージです。主に JSK の空中ロボット（マルチロータ）を対象としています。

## 概要

| コンポーネント | 説明 |
|---|---|
| **mujoco_model_generator.py** | YAML 設定ファイルに基づき xacro → URDF → MuJoCo XML の変換パイプラインを実行 |
| **mujoco_scene_composer.py** | 単体ロボットの MuJoCo XML を複数複製し、prefix 付きの multi-robot scene XML を生成 |
| **convert.py** | DAE (COLLADA) メッシュを OBJ に一括変換（テクスチャ・色情報を保持） |
| **dae_to_mujoco_body.py** | 単体の DAE を MuJoCo `<body>` XML + OBJ として出力するユーティリティ |
| **mujoco_ros_control (C++)** | MuJoCo をバックエンドとした `ros_control` ノード（シミュレーション実行） |

## 依存関係

### システム

- ROS (Melodic / Noetic)
- Python 2.7 または 3.x
- `libglfw3-dev`
- `xvfb`（ヘッドレス実行時）

### Python パッケージ

```bash
pip install trimesh pycollada pyyaml
```

### ROS パッケージ

- `xacro`, `controller_manager`, `hardware_interface`, `pluginlib`, `tf`, `urdf`, `sensor_msgs` 等（`package.xml` 参照）

## ビルド

```bash
cd ~/catkin_ws
catkin build mujoco_ros_control
# または
catkin_make --pkg mujoco_ros_control
```

ビルド時に MuJoCo 2.3.7 のバイナリが自動的にダウンロード・展開されます（`Makefile` の `download_unpack_build.mk` 経由）。

---

## mujoco_model_generator.py の使い方

### 基本コマンド

```bash
rosrun mujoco_ros_control mujoco_model_generator.py /absolute/path/to/config.yaml
```

> **注意**: 引数は YAML ファイルへの **絶対パス** で指定してください。

### YAML 設定ファイルの書き方

```yaml
package:
  - my_robot_description          # 変換対象の ROS パッケージ名（リスト）

my_robot_description:             # 上で指定したパッケージ名と同じキー
  meshdir: meshes                 # DAE メッシュが格納されたディレクトリ（パッケージルートからの相対パス）
  input:                          # xacro ファイルのパス（パッケージルートからの相対パス、リスト）
    - urdf/robot.urdf.xacro
  filename:                       # 出力先ディレクトリ名（<pkg>/mujoco/<filename>/ に生成される、リスト）
    - default
```

`input` と `filename` はリストで、複数モデルを一度に変換できます。

```yaml
package:
  - my_robot_description

my_robot_description:
  meshdir: meshes
  input:
    - urdf/robot.urdf.xacro
    - urdf/robot_with_arm.urdf.xacro
  filename:
    - default
    - with_arm
```

### MuJoCo専用のばね付き接触パッド（任意）

URDFを変更せず、生成したMuJoCoモデルだけにslide joint、ばね、球形接触geom、
接触・圧縮量センサを追加できます。`compliant_feet` を省略した既存設定の出力は
変わりません。

```yaml
my_robot_description:
  meshdir: meshes
  input: [urdf/robot.urdf.xacro]
  filename: [default]
  compliant_feet:
    parent_body: base_link
    axis: [0, 0, 1]                 # 正方向が圧縮方向
    joint_range: [-0.001, 0.015]   # m
    stiffness: 600.0               # N/m
    damping: 3.0                   # N s/m
    free_length: 0.024             # 取付点から球中心までの距離
    ball_radius: 0.012
    friction: [1.2, 0.02, 0.001]   # sliding, torsional, rolling
    feet:
      - {name: front, pos: [0.07, 0.0, -0.08]}
      - {name: rear,  pos: [-0.07, 0.0, -0.08]}
```

`name_prefix`を指定すると同じ機構を別の接触配列として追加できます。
Beeの把持では追加パッドを設けず、上記の物理的な4本の着地足をそのまま使用します。

各足について `spring_foot_<name>_touch`、`spring_foot_<name>_force`、
`spring_foot_<name>_compression`、`spring_foot_<name>_compression_velocity`
センサが生成されます。`force` は各siteのbodyと親bodyの間を伝わる3軸力です。
ばねjointをROSの通常の`joint_states`へ混ぜない場合は、ロボット側のsimulation設定へ
以下を追加します。未設定時は従来どおり全jointを登録・publishします。

```yaml
simulation:
  ignored_mujoco_joint_prefixes: [spring_foot_]
```

### 処理の流れ

```
1. DAE → OBJ 変換 (convert.py)
   メッシュディレクトリ内の全 .dae ファイルを .obj に変換。
   テクスチャや色情報は *_meta.json に保存される。

2. xacro → URDF 展開
   rosrun xacro xacro で xacro ファイルを URDF に変換。

3. URDF 前処理 (process_urdf)
   - メッシュパスを OBJ ファイルへ書き換え
   - package:// を実際のパスに解決
   - 複数サブメッシュへの分割対応
   - collision タグを visual からコピー
   - transmission タグから rotor / joint リストを抽出

4. MuJoCo コンパイル (generate_xml)
   MuJoCo 付属の compile コマンドで URDF → MuJoCo XML に変換。

5. MuJoCo XML 後処理 (process_xml)
   - ルートリンクの freejoint 追加（浮遊ベース）
   - ロータをサイト化（推力アクチュエータ用）
   - アクチュエータ定義（ロータ: motor, ジョイント: position）
   - IMU センサ定義（accelerometer, gyro, magnetometer）
   - テクスチャ・マテリアル適用
   - world.xml のインクルード

6. 一時ファイルのクリーンアップ
   変換で生成された中間 OBJ / meta JSON を削除。
```

### 出力

```
<パッケージ>/mujoco/<filename>/
  ├── robot.urdf    # 変換済み URDF（中間ファイル）
  ├── robot.xml     # MuJoCo モデル（最終出力）
  └── *.obj         # メッシュファイル群
```

### CMake からの自動実行

ロボットパッケージの `CMakeLists.txt` で自動変換を設定できます：

```cmake
find_package(catkin REQUIRED COMPONENTS mujoco_ros_control)
include(${mujoco_ros_control_DIR}/model_convert.cmake)
mujoco_model_convert(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/config/mujoco.yaml)
```

ビルド時に `mujoco_model_generator.py` が自動実行され、`<パッケージ>/mujoco/` 以下にモデルが生成されます。

---

## シミュレーションの実行

```bash
roslaunch mujoco_ros_control mujoco.launch mujoco_model:=/path/to/robot.xml
```

### Launch 引数

| 引数 | デフォルト | 説明 |
|---|---|---|
| `robot_ns` | `/` | ロボットの名前空間 |
| `headless` | `false` | `true` で GUI なし実行（CI 等） |
| `mujoco_model` | `""` | MuJoCo XML モデルの絶対パス |
| `render_fps` | `30.0` | simulation time 1秒あたりの描画回数。物理step周期は変更しない |
| `vsync` | `false` | `true` で画面更新をVSyncへ同期 |
| `render_shadows` | `false` | shadow描画を有効化 |
| `render_reflections` | `false` | reflection描画を有効化 |
| `window_width` | `960` | viewer幅 [pixel] |
| `window_height` | `720` | viewer高さ [pixel] |

WSL/WSLgでは描画と物理計算が同じthreadで実行されるため、OpenGLのbuffer転送や
VSync待ちがsimulation timeも停止させます。multi-robot launchでは描画負荷を抑えるため
既定を`render_fps:=10`、`vsync:=false`、shadow/reflectionなし、`640x480`としています。
画質を上げる場合:

```bash
roslaunch bee mujoco_multi_module.launch headless:=false \
  render_fps:=15 vsync:=false render_shadows:=true \
  render_reflections:=true window_width:=800 window_height:=600
```

MuJoCo viewerが不要なら`headless:=true`が最速です。RViz表示は別processなので、
MuJoCoをheadlessにしたまま`launch_rviz:=true`を使用できます。

### 複数ロボットを同一 MuJoCo に入れる

1 台ぶんの `robot.xml` を生成したあと、`mujoco_scene_composer.py` で複数機 scene を作ります。

```bash
rosrun mujoco_ros_control mujoco_scene_composer.py /absolute/path/to/scene.yaml
```

`scene.yaml` の例:

```yaml
scene_name: bee_scene_4
source_model: ../mujoco/bee/robot.xml
output_model: ../mujoco/bee_scene_4.xml
jacobian: dense  # 4台 + free objectではMuJoCo 2.3.xの安定性のため推奨

robots:
  - name: bee1
    pos: [0.0, 0.0, 0.0]
  - name: bee2
    pos: [1.2, 0.0, 0.0]
  - name: bee3
    pos: [0.0, 1.2, 0.0]
  - name: bee4
    pos: [1.2, 1.2, 0.0]

objects:
  - name: grasp_pedestal
    type: cylinder
    size: [0.20, 0.65]  # [radius, height]
    pos: [0.6, 0.6, 0.325]
    friction: [1.2, 2.0, 0.001]
  - name: grasp_prism
    type: triangular_prism
    # 正三角形断面をXY、高さ方向をZにした自由物体
    size: [0.80, 0.30]  # [triangle_side, height]
    mass: 1.00
    pos: [0.6, 0.6, 0.802]
    friction: [1.2, 2.0, 0.001]
```

`source_model` と `output_model` は、YAML ファイル基準の相対パスでも絶対パスでも指定できます。

scene composer は各ロボットの `body`、`joint`、`actuator`、`site`、`sensor`、`mesh`、`material`、`texture` 名に `bee1_` のような prefix を付け、初期位置をずらした 1 つの scene XML を出力します。
`objects` はprefixを付けずsceneへ1回だけ追加されます。`triangular_prism` はinlineの
convex meshと`freejoint`で生成されるため、床やロボットと接触し、把持後に持ち上げられます。

実行時は `robot_namespaces` を与えると、`mujoco_ros_control` が namespace ごとに独立した `RobotHWSim` と `controller_manager` を作ります。

```bash
roslaunch mujoco_ros_control mujoco_multi.launch \
  mujoco_model:=/path/to/bee_scene_4.xml \
  robot_namespaces:="['bee1', 'bee2', 'bee3', 'bee4']"
```

このとき、各 namespace は次のように分離されます。

- `/bee1/joint_states`, `/bee1/mujoco/ctrl_input`
- `/bee2/joint_states`, `/bee2/mujoco/ctrl_input`
- `/bee3/joint_states`, `/bee3/mujoco/ctrl_input`
- `/bee4/joint_states`, `/bee4/mujoco/ctrl_input`

各ロボット側の actuator / sensor 名は `<namespace>_` prefix 付きで scene XML に埋め込まれ、HWSim は自分の prefix に一致する要素だけを扱います。

bee パッケージでは、Gazebo の `robot_id` ベースの起動に寄せた multi-robot launcher も使えます。

```bash
roslaunch bee mujoco_multi_module.launch robot_count:=3
```

既定ではMuJoCo backendを`headless:=true`で動かし、共有RVizを1つだけ起動します。
RVizには`bee1`〜`bee3`、三角柱、pedestal、および各Beeの物理的な4本のばね脚が
表示されます。ばね脚のTFにはMuJoCoで計測した圧縮量を反映します。RVizも不要な場合は
`launch_rviz:=false`を指定します。シーンMarkerのtopicは
`/mujoco/grasp_scene_markers`です。

このlauncherは既定で、高さ0.65 m・半径0.20 mのpedestalと、3側面から把持できる
正三角柱（一辺0.80 m、高さ0.30 m、質量1.0 kg）を追加します。三角柱の中心は
`(0, 1, 0.802)`です。pedestalを使わず床へ置く場合は
`spawn_object_pedestal:=false`を指定します。
物体なしの従来sceneは`spawn_object:=false`で起動できます。
台座上でのyaw回転を抑えるねじり摩擦は
`object_torsional_friction`（既定2.0）で調整できます。

既定の3台は三角柱の各側面法線上へ配置されます。接近前に各Beeの
`final_target_baselink_rpy`へroll=1.57 radを指令し、横倒しにした機体の
物理的な4本の着地足を側面へ向けます。
初期の面からの距離は`grasp_staging_clearance`（既定0.80 m）で調整できます。
4点の力は各namespaceの`mujoco/grasp_forces`（機体fc座標）と
`mujoco/grasp_forces_world`（world座標）へ
`[spring_foot_front, spring_foot_rear, spring_foot_left, spring_foot_right]`
の順でpublishされます。これらの`grasp_*` topicは物理足センサの互換aliasです。
4本の接触状態は`sensor_msgs/JointState`型の`mujoco/grasp_contact_states`へ
同じ順序でpublishされ、`position`がslide joint圧縮量[m]、`velocity`が圧縮速度[m/s]、
`effort`がMuJoCo touchセンサの法線接触力[N]です。touchとforceは同じsiteを使いますが、
前者はsite体積内の接触法線力のスカラー和、後者は親子body間の3軸伝達力であり別センサです。
Beeの`mujoco/external_wrench`へ`geometry_msgs/WrenchStamped`をpublishすると、
world座標の外力・トルクがroot bodyへ加算されます。指令が0.1秒以上途切れると自動でゼロになり、
root pose/velocityを直接変更せずMuJoCoの運動方程式と接触拘束を通して運動します。

この launcher は次を自動で行います。

- `bee/config/mujoco_model.yaml` から単体 `robot.xml` を必要に応じて生成
- 台数に応じて multi-robot scene XML を生成
- MuJoCo backend を 1 回だけ起動
- `bee1` から `beeN` までの `bringup.launch` を個別 namespace で起動

`grid_cols` を指定すると格子状に配置できます。`grid_cols:=0` のときは 1 列配置です。

---

## 補助スクリプト

### convert.py

DAE メッシュの一括 OBJ 変換。通常は `mujoco_model_generator.py` から内部的に呼ばれます。

```bash
python convert.py /path/to/mesh_directory
```

### dae_to_mujoco_body.py

単体の DAE ファイルを MuJoCo の `<body>` XML として出力します。障害物などをシーンに追加する際に便利です。

```bash
python dae_to_mujoco_body.py input.dae [output_dir] [--body-name NAME] [--pos "x y z"] [--euler "r p y"]
```

生成された `body.xml` を MuJoCo モデル内で `<include file="..."/>` でインクルードできます。

### mujoco_scene_composer.py

単体ロボット用の `robot.xml` から multi-robot scene XML を生成します。

```bash
python mujoco_scene_composer.py /path/to/scene.yaml
```

---

## URDF 側の要件

`mujoco_model_generator.py` が正しく動作するために、URDF/xacro に以下の定義が必要です：

- **`<transmission>` タグ**: アクチュエータの種類を指定
  - `<hardwareInterface>RotorInterface</hardwareInterface>` → ロータ（推力）
  - `<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>` → 関節（位置制御）
- **`<baselink>` タグ**: フライトコントローラ（IMU）の搭載リンク名を `name` 属性で指定
- **`<m_f_rate>` タグ**: ロータのモーメント/推力比を `value` 属性で指定
- **ルートリンク**: `root` という名前の親リンクから接続された子リンクがルートボディになる

---

## ディレクトリ構成

```
mujoco_ros_control/
├── CMakeLists.txt
├── Makefile                          # MuJoCo バイナリのダウンロード
├── package.xml
├── mujoco_robot_hw_sim_plugin.xml    # pluginlib 定義
├── cmake/
│   └── model_convert.cmake           # catkin 自動変換マクロ
├── config/
│   ├── world.xml                     # デフォルトワールド（地面・照明・スカイボックス）
│   └── filter.mxl                    # MeshLab デシメーションフィルタ（レガシー）
├── include/mujoco_ros_control/       # C++ ヘッダ
├── launch/
│   └── mujoco.launch
├── scripts/
│   ├── mujoco_model_generator.py     # メイン変換スクリプト
│   ├── convert.py                    # DAE → OBJ 変換
│   └── dae_to_mujoco_body.py         # DAE → MuJoCo body XML
├── src/                              # C++ ソース
│   ├── mujoco_ros_control.cpp
│   ├── mujoco_default_robot_hw_sim.cpp
│   └── mujoco_visualization_utils.cpp
└── build/
    └── mujoco-2.3.7/                # ダウンロードされた MuJoCo バイナリ
```

## ライセンス

BSD
