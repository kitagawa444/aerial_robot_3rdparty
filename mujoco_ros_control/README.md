# mujoco_ros_control

ROS (catkin) パッケージ。URDF/xacro で記述されたロボットモデルを MuJoCo シミュレーション用の XML に自動変換し、`ros_control` インターフェースで制御するためのパッケージです。主に JSK の空中ロボット（マルチロータ）を対象としています。

## 概要

| コンポーネント | 説明 |
|---|---|
| **mujoco_model_generator.py** | YAML 設定ファイルに基づき xacro → URDF → MuJoCo XML の変換パイプラインを実行 |
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
