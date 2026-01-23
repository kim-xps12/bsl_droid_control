# Biped Description パッケージ技術資料

## 概要

`biped_description`は、逆関節（Digitigrade/鳥脚型）二脚ロボットのURDF記述とRViz2可視化ツールを提供するROS 2パッケージです。

> 📖 **環境構築・クイックスタート**については[プロジェクトルートのREADME](../../../../README.md)を参照してください。

### 特徴

- **逆関節構造**: 膝が前方に曲がる鳥脚型ロボット
- **10自由度**: 片脚5関節 × 2脚（左右対称）
- **カスタムGUI**: 左右2列表示のPyQt5ベース関節操作GUI

## ファイル構成

```
biped_description/
├── CMakeLists.txt              # ビルド設定
├── package.xml                 # パッケージメタデータ
├── README.md                   # 簡易説明
├── doc/
│   ├── technical_specification.md  # 本ドキュメント
│   ├── robot_structure.drawio      # ロボット構造図
│   ├── tf_tree.drawio              # TFツリー図
│   └── node_architecture.drawio    # ノード構成図
├── urdf/
│   └── biped_digitigrade.urdf.xacro  # ロボットモデル定義
├── launch/
│   ├── display.launch.py           # 標準GUI起動
│   └── display_custom.launch.py    # カスタム2列GUI起動
├── rviz/
│   └── biped_display.rviz          # RViz2設定
└── scripts/
    └── joint_gui.py                # カスタムGUIスクリプト
```

## ロボット仕様

### 物理パラメータ

| パラメータ | 値 [m] | 説明 |
|-----------|--------|------|
| hip_width | 0.16 | 左右の股関節間距離 |
| hip_yaw_length | 0.03 | ヨー軸リンク長 |
| hip_roll_length | 0.02 | ロール軸リンク長 |
| thigh_length | 0.18 | 大腿部長さ |
| shank_length | 0.20 | 下腿部長さ |
| foot_length | 0.10 | 足部長さ |

※ パラメータは `ref/example_gait.py` の `link_lengths` と一致

### 関節定義

#### 左脚 (j1x系列)

| 関節名 | ID | 軸 | 可動範囲 | 説明 |
|--------|----|----|----------|------|
| left_hip_yaw_joint | j11 | Z | ±30° | 腰ヨー軸 |
| left_hip_roll_joint | j12 | X | ±20° | 股関節ロール軸 |
| left_hip_pitch_joint | j13 | Y | ±90° | 股関節ピッチ軸 |
| left_knee_pitch_joint | j14 | Y | -120°~0° | 膝ピッチ軸（逆関節） |
| left_ankle_pitch_joint | j15 | Y | -45°~135° | 足首ピッチ軸 |

#### 右脚 (j2x系列)

| 関節名 | ID | 軸 | 可動範囲 | 説明 |
|--------|----|----|----------|------|
| right_hip_yaw_joint | j21 | Z | ±30° | 腰ヨー軸 |
| right_hip_roll_joint | j22 | X | ±20° | 股関節ロール軸 |
| right_hip_pitch_joint | j23 | Y | ±90° | 股関節ピッチ軸 |
| right_knee_pitch_joint | j24 | Y | -120°~0° | 膝ピッチ軸（逆関節） |
| right_ankle_pitch_joint | j25 | Y | -45°~135° | 足首ピッチ軸 |

### 逆関節（Digitigrade）の特徴

```
通常の人型ロボット（Plantigrade）:
    ┌─┐
    │ │ ← 胴体
    └┬┘
     │
     ├─○ hip_pitch
     │
     │  thigh
     │
     ├─○ knee_pitch (後方に曲がる: 正の角度)
     │
     │  shank
     │
     └─○ ankle_pitch
        │
       ═══ foot (前方に伸びる)

逆関節ロボット（Digitigrade）:
    ┌─┐
    │ │ ← 胴体
    └┬┘
     │
     ├─○ hip_pitch
     │
     │  thigh
     │
     ○─┤ knee_pitch (前方に曲がる: 負の角度)
       │
       │  shank
       │
       ○─┤ ankle_pitch
         │
        ═══ foot (後方に伸びる)
```

**重要**: 膝関節の可動範囲が `-120°~0°` であることに注意。負の角度で前方に曲がる。

## URDF構造

### xacroマクロ

`biped_digitigrade.urdf.xacro` では、左右対称の脚を生成するためにxacroマクロを使用：

```xml
<xacro:macro name="leg" params="side reflect">
  <!-- side: "left" or "right" -->
  <!-- reflect: 1 (left) or -1 (right) -->
</xacro:macro>

<xacro:leg side="left" reflect="1"/>
<xacro:leg side="right" reflect="-1"/>
```

### リンクツリー

```
base_link
├── left_hip_yaw_link
│   └── left_hip_roll_link
│       └── left_thigh_link
│           └── left_shank_link
│               └── left_foot_link
└── right_hip_yaw_link
    └── right_hip_roll_link
        └── right_thigh_link
            └── right_shank_link
                └── right_foot_link
```

## ノード構成

### display_custom.launch.py

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| robot_state_publisher | robot_state_publisher | URDFを読み込み、/tfを配信 |
| joint_gui | (Python script) | PyQt5カスタムGUI、/joint_statesを配信 |
| rviz2 | rviz2 | 3D可視化 |

### display.launch.py

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| robot_state_publisher | robot_state_publisher | URDFを読み込み、/tfを配信 |
| joint_state_publisher_gui | joint_state_publisher_gui | 標準GUI、/joint_statesを配信 |
| rviz2 | rviz2 | 3D可視化 |

### トピック一覧

| トピック名 | 型 | 方向 | 説明 |
|-----------|-----|------|------|
| `/robot_description` | `std_msgs/String` | Pub | URDF文字列 |
| `/joint_states` | `sensor_msgs/JointState` | Pub | 関節状態 |
| `/tf` | `tf2_msgs/TFMessage` | Pub | 座標変換 |
| `/tf_static` | `tf2_msgs/TFMessage` | Pub | 静的座標変換 |

## カスタムGUI (joint_gui.py)

### 機能

| ボタン | 機能 |
|--------|------|
| Reset All | 全関節を0°にリセット |
| Center Pose | 自然な立位姿勢（膝を軽く曲げた状態） |

### 実装詳細

- **フレームワーク**: PyQt5
- **更新レート**: 50Hz
- **レイアウト**: 左右2列（Left Leg / Right Leg）
- **スライダー**: 各関節の可動範囲に対応

## 関連ドキュメント

- [ルートREADME](../../../../README.md) - 環境構築・クイックスタート
- [example_gait.py](../../../../ref/example_gait.py) - 歩容生成の参考実装
- [robstride_hardware](../../robstride_hardware/) - 実機制御用ハードウェアインターフェース

## 変更履歴

| 日付 | 変更内容 |
|------|----------|
| 2026-01-23 | 初版作成。URDF、カスタムGUI実装 |
