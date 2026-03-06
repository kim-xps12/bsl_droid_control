# BSL-Droid ROS 2 歩行制御システム — 要件定義

## 概要

BSL-Droid（逆関節10自由度二脚ロボット）をLogitech F710rゲームパッドで操縦するROS 2歩行制御システムの要件を定義する。RL学習済みポリシー（droid-walking-omni系、50次元観測・10次元行動）が速度指令から歩容を生成し、ros2_controlを介して10個のRS02モータを制御する。

本システムはシミュレーション検証後に実機（Jetson Orin Nano Super）へ展開する段階的アプローチをとる。Phase 1（RViz可視化）→ Phase 2（安全監視整備）→ Phase 3（実機展開）の順に構築する。

## 機能要件

### FR-01: ゲームパッド速度指令入力（Must）

- Logitech F710r ゲームパッドから速度指令を入力する
- 軸マッピング（DirectInputモード）:
  - 左スティック上下（axis 1）→ 前後速度 `lin_vel_x`
  - 左スティック左右（axis 0）→ 横速度 `lin_vel_y`
  - 右スティック左右（axis 2）→ yaw角速度 `ang_vel_yaw`
- デッドゾーン: 0.08（スティック中心のドリフト防止）
- 速度上限はパラメータで設定可能とする（デフォルト値は学習時の`commands_scale`に準拠）
- ROS 2標準の`joy_node`および`teleop_twist_joy`を使用し、デバイスhotplug・ROS 2エコシステム統合を確保する
- 参照: `rl_ws/biped_walking/biped_eval_gamepad.py`の軸マッピングに準拠

### FR-02: RLポリシーによる歩容生成（Must）

- 学習済みRLポリシー（droid-walking-omni系）で速度指令から10関節の目標位置を50Hzで生成する
- 観測ベクトルは50次元で、`rl_ws/biped_walking/envs/droid_env_unitree.py:714-730`と完全一致させる
  - `base_lin_vel` [3]: ベース線速度（vizモードではモック [0,0,0]、実機ではIMU推定値）
  - `base_ang_vel` [3]: ベース角速度（vizモードではモック [0,0,0]、実機ではIMU角速度）
  - `projected_gravity` [3]: 射影重力ベクトル（vizモードではモック [0,0,-1]、実機ではIMU姿勢から計算）
  - `commands` [3]: 速度指令（/cmd_velから取得、×commands_scale）
  - `dof_pos - default_dof_pos` [10]: 関節偏差（前回出力から計算、×obs_scales.dof_pos）
  - `dof_vel` [10]: 関節速度（差分計算、×obs_scales.dof_vel）
  - `actions` [10]: 前回のアクション出力
  - `gait_phase_sin/cos` [2]: 歩容位相（内部クロック50Hz、周期は学習時と同一）
  - `leg_phase` [2]: 脚位相（gait_phaseから導出）
  - `feet_pos_z` [2]: 足先Z座標（FK計算 or 定数）
  - `contact_state` [2]: 接地状態（vizモードではモック [1,1]、実機ではトルク推定）
- アクション→関節位置変換: `target = action * action_scale + default_dof_pos`
  - `action_scale` = 0.25（`droid_env_unitree.py:487`）
  - `default_dof_pos`: hip_pitch=60°, knee=-100°, ankle=45°（`droid_train_omni_v21.py:120-122`）
- モデルはPyTorch MLP（512-256-128）。Pythonでの50Hz推論はレイテンシ要件を満たす（libtorch化は時期尚早）
- モデル・設定ファイルは`rl_ws/logs/droid-walking-omni-v{N}/`からロードする

### FR-03: ros2_controlによるモータ制御（Must）

- ros2_controlの`ForwardCommandController`を介して10個のRS02モータを200Hzで位置制御する
- RLポリシー出力（50Hz）をForwardCommandControllerが200Hzで補間しハードウェアPD制御を適用する
  - ハードウェアPDゲイン: kp=35, kd=2（robstride_hardware設定に準拠）
- 10関節の構成:
  - 左脚: left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee_pitch, left_ankle_pitch
  - 右脚: right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee_pitch, right_ankle_pitch
- CAN bus（1Mbps）経由でRS02モータと通信（robstride_hardwareが担当）
- 参照: robstride_hardware技術仕様

### FR-04: RViz2リアルタイム可視化（Must）

- `/joint_states`をrobot_state_publisher経由でRViz2に反映し、ロボットの関節状態をリアルタイム可視化する
- vizモード（Phase 1）ではRLポリシーノードが直接`/joint_states`をpublishする
- controlモード（Phase 3）ではjoint_state_broadcasterが実エンコーダデータをpublishする
- MacBookからDDS経由で/joint_statesを受信しRViz2で表示する（分散構成）
- 参照: biped_description技術仕様

### FR-05: 緊急停止（Must）

- ゲームパッドの専用ボタン（STARTボタン等）で即座に緊急停止を発行する
- 緊急停止時は全関節を現在位置で保持（急なトルクオフは転倒リスクあり）
- `/emergency_stop`トピック（std_msgs/Bool）で全ノードに通知する
- 緊急停止からの復帰には明示的な操作（ボタン再押下等）を要求する

### FR-06: 安全監視（Must）

- 200Hzの高頻度で以下の安全チェックを実行する:
  - **関節角度リミット**: URDFに定義された各関節のlimit値の100%を閾値とする
  - **関節速度リミット**: RS02モータ仕様の最大速度（~25 rad/s）を閾値とする
  - **姿勢異常**: Roll/Pitch > 60°で警告→緊急停止（IMU実装後）
  - **転倒検出**: ベース高さ < 0.1mで緊急停止（IMU実装後）
- `/safety_status`トピックで監視結果を常時publishする
- 違反検出時は`/emergency_stop`をpublishし、全アクチュエータを安全状態に遷移させる
- 参照: 次期ノード設計 Safety Monitor

### FR-07: ゲームパッド切断時のフェイルセーフ（Must）

- ゲームパッド切断（`joy_node`のタイムアウト）を検出したら速度指令をゼロにする
- 具体的にはTwistメッセージの全フィールドを0.0で送信し、ロボットを立位姿勢に戻す
- 切断後のRLポリシーは「速度指令ゼロ」として動作し続ける（急停止ではなく緩やかに停止）

## 非機能要件

### NFR-01: RLポリシー推論レイテンシ

- RLポリシーの1回の推論（観測構築→PyTorch forward→アクション出力）を10ms以内に完了する
- 50Hzの制御周期（20ms）に対し、推論以外の処理（通信・観測取得）に10msの余裕を持たせる
- Jetson Orin Nano SuperのCPU上で達成すること（GPU推論は不要、MLPモデルのため）
- 推論時間はRLPolicyState.msgのinference_time_msフィールドで常時モニタリングする

### NFR-02: エンドツーエンドレイテンシ

- ゲームパッド操作からアクチュエータ動作開始までの遅延を40ms以内とする
- レイテンシの内訳目安:
  - ゲームパッド→joy_node: ~5ms（USB polling）
  - joy_node→teleop_twist_joy→cmd_vel: ~2ms（ROS 2 intra-process）
  - cmd_vel→RLポリシー推論→position commands: ~10ms
  - ForwardCommandController→robstride_hardware→CAN: ~5ms
  - 合計目安: ~22ms（マージン込み40ms以内）
- 参照: robstride_hardware レイテンシ分析

### NFR-03: 安全監視レイテンシ

- 安全監視ノードは200Hzで動作し、1回の処理を1ms以内に完了する
- 安全違反の検出から緊急停止信号の発行まで5ms以内とする
- SCHED_FIFO優先度45での実行を想定（robstride_hardwareの50より低く、一般ノードより高い）
- 参照: 次期ノード設計 性能要件

### NFR-04: 動作プラットフォーム

- 実機動作環境: Jetson Orin Nano Super（8GB RAM, ARM64, Ubuntu 22.04）
- 開発・可視化環境: MacBook（osx-arm64）— RViz可視化・デバッグのみ
- ROS 2 Jazzy（pixi経由、RoboStack提供）を使用
- MacBookではros2_controlは利用不可（RoboStack JazzyのmacOS ARM64ビルド未提供）
- PyTorchはJetson上ではJetPack付属のシステムPyTorchをpip経由で利用する
- 参照: 分散システムアーキテクチャ

## 安全要件

### SR-01: 過度傾斜による緊急停止

- IMUで計測したRoll/Pitchが60°を超えた場合、即座に緊急停止を発行する
- 閾値60°は転倒復帰不能な姿勢に至る前のマージンとして設定
- Phase 1（vizモード）ではIMU非搭載のためこのチェックは無効、Phase 3（実機）で有効化する

### SR-02: 転倒検出による緊急停止

- IMUと運動学計算から推定したベース高さが0.1m未満になった場合、緊急停止を発行する
- BSL-Droidの立位時のベース高さ（約0.35m）に対し、0.1mは明確な転倒状態を示す
- Phase 1では無効、Phase 3で有効化する

### SR-03: 関節リミット超過による停止

- 各関節の角度がURDFに定義されたlimit値の100%に達した場合、該当関節を停止する
- 全Phase共通で有効（エンコーダデータまたはRLポリシー出力から検知）
- 参照: biped_description技術仕様 関節定義

### SR-04: デッドマンスイッチ

- F710rのLBボタン（button 4）をデッドマンスイッチとして使用する
- LBボタン押下中のみ歩行指令を送信、離すと速度指令ゼロ（立位姿勢に遷移）
- 常に物理的な人間の意思確認を必要とし、制御の暴走を防ぐ

