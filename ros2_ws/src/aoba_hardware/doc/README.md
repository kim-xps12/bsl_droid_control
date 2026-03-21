# aoba_hardware クイックリファレンス

詳細設計は `doc/design/ros2_walking/modules/aoba_hardware.md` を参照。

## CAN バス・モータ ID マッピング

| CAN バス | 関節名 | モータ ID |
|---|---|---|
| can1 | left_hip_yaw_joint | 11 |
| can1 | left_hip_roll_joint | 12 |
| can1 | left_hip_pitch_joint | 13 |
| can1 | left_knee_pitch_joint | 14 |
| can1 | left_ankle_pitch_joint | 15 |
| can2 | right_hip_yaw_joint | 21 |
| can2 | right_hip_roll_joint | 22 |
| can2 | right_hip_pitch_joint | 23 |
| can2 | right_knee_pitch_joint | 24 |
| can2 | right_ankle_pitch_joint | 25 |

起動コマンドはプロジェクトルートの `README.md` を参照してください。
