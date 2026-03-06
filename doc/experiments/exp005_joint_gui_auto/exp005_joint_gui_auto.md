# EXP005: 関節GUIの自動可動範囲取得機能

## 概要

| 項目 | 内容 |
|------|------|
| 実験ID | EXP-005 |
| 目的 | スライダーGUIの関節可動範囲をURDFから自動取得し、ハードコードを排除 |
| 対象ファイル | `joint_gui.py`, `display_bsl_droid_simplified.launch.py` |
| 実行環境 | ROS 2 + PyQt5 |

## 背景

従来の`joint_gui.py`では、各関節の可動範囲（最小/最大角度）がスクリプト内にハードコードされていた。

```python
# 旧実装（ハードコード）
LEFT_JOINTS = [
    ('left_hip_yaw_joint', -30, 30),
    ('left_hip_roll_joint', -25, 25),
    ('left_hip_pitch_joint', -120, 60),
    ('left_knee_pitch_joint', -135, 0),
    ('left_ankle_pitch_joint', -60, 60),
]
```

この設計には以下の問題があった：

1. **二重管理**: URDFの`<limit>`タグと`joint_gui.py`の両方で可動範囲を管理する必要がある
2. **同期の手間**: URDFを変更するたびにGUIスクリプトも手動で更新が必要
3. **エラーの原因**: URDFとGUIの値が不一致になりやすい

## 解決策

`robot_state_publisher`が公開する`robot_description`パラメータからURDFを取得し、XMLパースして関節の可動範囲を自動抽出する。

### アーキテクチャ

```
┌─────────────────────┐
│  xacro file         │
│  (URDF定義)          │
└─────────┬───────────┘
          │ xacro展開
          ▼
┌─────────────────────┐
│ robot_state_publisher│
│ (robot_description) │
└─────────┬───────────┘
          │ GetParameters Service
          ▼
┌─────────────────────┐
│    joint_gui.py     │
│  (XMLパース→GUI生成) │
└─────────────────────┘
```

### データフロー

1. `robot_state_publisher`が起動し、xacroを処理してURDFを`robot_description`パラメータに設定
2. 1秒後に`joint_gui.py`が起動（`TimerAction`による遅延）
3. GUIが`/robot_state_publisher/get_parameters`サービスを呼び出し
4. 取得したURDF XMLをパースして`<joint type="revolute">`の`<limit>`を抽出
5. 抽出した可動範囲でスライダーを動的生成

## 実装詳細

### 変更ファイル

| ファイル | 変更内容 |
|----------|----------|
| `ros2_ws/src/biped_description/scripts/joint_gui.py` | 自動可動範囲取得版に置き換え |
| `ros2_ws/src/biped_description/launch/display_bsl_droid_simplified.launch.py` | `TimerAction`による遅延起動を追加 |

### 主要な関数

#### `get_robot_description()`

```python
def get_robot_description(node: Node, timeout_sec: float = 10.0) -> str:
    """robot_state_publisherからrobot_descriptionパラメータを取得する"""
    client = node.create_client(
        GetParameters,
        '/robot_state_publisher/get_parameters'
    )
    # サービス呼び出し...
    return response.values[0].string_value
```

#### `parse_joint_limits_from_urdf()`

```python
def parse_joint_limits_from_urdf(urdf_xml: str) -> dict:
    """URDF XMLから関節の可動範囲を抽出する"""
    joints = {}
    root = ET.fromstring(urdf_xml)
    for joint in root.findall('.//joint'):
        joint_type = joint.get('type')
        if joint_type not in ('revolute', 'prismatic'):
            continue
        name = joint.get('name')
        limit = joint.find('limit')
        if limit is not None:
            lower = float(limit.get('lower', 0))
            upper = float(limit.get('upper', 0))
            joints[name] = (lower, upper)
    return joints
```

### launchファイルの変更

```python
# 旧実装
custom_joint_gui = ExecuteProcess(
    cmd=['python3', joint_gui_script],
    ...
)

# 新実装（遅延起動）
custom_joint_gui = TimerAction(
    period=1.0,  # 1秒待機してからGUIを起動
    actions=[
        ExecuteProcess(
            cmd=['python3', joint_gui_script],
            ...
        )
    ]
)
```

## GUI改善点

### 可動範囲の表示

各スライダーに可動範囲をグレー文字で表示するようになった。

```
┌─────────────────────┐
│    L:hip_yaw        │
│   [-30° ~ 30°]      │  ← 新規追加
│ ──────●────────     │
│       0.0°          │
└─────────────────────┘
```

### 関節の表示順

物理的な接続順（股関節→足首）で表示するためのソートキーを実装。

```python
JOINT_ORDER = [
    'hip_yaw',
    'hip_roll',
    'hip_pitch',
    'knee_pitch',
    'ankle_pitch',
]
```

## 使用方法

```bash
cd ros2_ws
pixi run colcon build --packages-select biped_description --symlink-install
pixi run ros2 launch biped_description display_bsl_droid_simplified.launch.py
```

## 利点

| 項目 | 旧実装 | 新実装 |
|------|--------|--------|
| 可動範囲の管理 | URDFとGUIの二重管理 | URDFのみ（Single Source of Truth） |
| URDF変更時 | GUIスクリプトも手動更新 | 自動反映 |
| 汎用性 | 特定のロボット専用 | 任意のURDFに対応 |
| 可動範囲の確認 | コード内を参照 | GUIに表示 |

## 注意点

### 起動順序

`robot_state_publisher`が先に起動している必要があるため、`TimerAction`で1秒の遅延を入れている。この遅延が不十分な場合、GUIはエラーダイアログを表示して終了する。

### 対応する関節タイプ

現在の実装では`revolute`（回転関節）と`prismatic`（直動関節）のみ対応。`continuous`（連続回転）関節は可動範囲が定義されないためスキップされる。

## 今後の改善案

1. **起動順序の改善**: `TimerAction`ではなく、`robot_state_publisher`の起動完了を検知するイベントベースの起動
2. **ホットリロード**: URDFが変更された場合にGUIを再読み込み
3. **複数ロボット対応**: 名前空間やパラメータ指定による柔軟な対応

## 関連ドキュメント

- [EXP003: BD-1風二脚ロボットURDFモデル設計](../exp003_biped_urdf/exp003_biped_urdf.md)
- [ROS 2 robot_state_publisher](https://docs.ros.org/en/jazzy/p/robot_state_publisher/)
