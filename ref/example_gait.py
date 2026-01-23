# control_process.py
import zmq
import numpy as np
import time

class BipedController:
    def __init__(self):
        # ZeroMQ設定
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://127.0.0.1:5555")
        time.sleep(0.1)
        
        # ロボットパラメータ
        # 脚構造: 片脚5関節 × 2脚（左右対称）
        # 逆関節（digitigrade/鳥脚型）: 膝が前方に突出し、足部が後方に伸びる
        # 左脚: j11(腰ヨー), j12(股関節ロール), j13(股関節ピッチ), j14(膝ピッチ), j15(足首ピッチ)
        # 右脚: j21(腰ヨー), j22(股関節ロール), j23(股関節ピッチ), j24(膝ピッチ), j25(足首ピッチ)
        self.link_lengths = {
            'hip_width': 0.16,     # 左右の股関節間距離（Y方向）[m]
            'hip_yaw': 0.03,       # ヨー軸リンク長（腰ヨー→股関節ロール）[m]
            'hip_roll': 0.02,      # ロール軸リンク長（股関節ロール→ピッチ）[m]
            'thigh': 0.18,         # 大腿部長さ（股関節ピッチ→膝ピッチ）[m]
            'shank': 0.20,         # 下腿部長さ（膝ピッチ→足首ピッチ）[m] - 逆関節では少し長め
            'foot': 0.10           # 足部長さ（足首ピッチ→足先、後方に伸びる）[m]
        }
        
        # 歩容パラメータ
        self.step_height = 0.04   # 足上げ高さ [m] (Z方向)
        self.step_length = 0.08   # ストライド長さ [m] (X方向)
        self.step_frequency = 0.5  # 歩行周波数 [Hz]
        
        # 基準姿勢: 膝を少し曲げた状態で立つ
        # 脚全長の90%程度を使用（自然な屈身状態）
        self.leg_extension_ratio = 0.90
        
        # 座標系の基準点:
        # - 原点 (0, 0, 0): XY平面上の胴体中心、Z=0は地面
        # - 胴体高さ: 接地状態で足先がZ=0になるよう自動計算
        self._calculate_body_height()
        
    def _calculate_body_height(self):
        """
        胴体高さを脚の構造から自動計算
        
        接地状態（脚伸展率に基づく屈曲姿勢）で足先がZ=0になるよう設定
        
        座標系:
        - Z=0: 地面
        - 胴体中心: (0, 0, body_height)
        - 足先: 接地時に (x, y, 0)
        """
        L = self.link_lengths
        
        # 大腿部+下腿部の垂直成分（伸展率を考慮）
        thigh_shank_vertical = (L['thigh'] + L['shank']) * self.leg_extension_ratio
        
        # 逆関節の接地姿勢を計算して足部のZ方向寄与を求める
        # 逆運動学で接地姿勢の角度を取得
        target_x = 0  # 接地時は股関節真下
        target_z = -thigh_shank_vertical
        hip_pitch, knee_pitch = self.inverse_kinematics_2link(
            target_x, target_z, L['thigh'], L['shank']
        )
        
        # 足首角度
        ankle_pitch = self.calculate_foot_angle(hip_pitch, knee_pitch)
        total_angle = np.radians(hip_pitch + knee_pitch + ankle_pitch)
        
        # 足部のZ方向成分（後方に伸びる逆関節の足部）
        foot_z = L['foot'] * np.sin(total_angle)
        
        # 胴体高さ = ロール軸リンク + 大腿下腿の垂直成分 + 足部のZ成分
        self.body_height = L['hip_roll'] + thigh_shank_vertical + foot_z
        
        print(f"座標系情報:")
        print(f"  - 原点: 地面中央 (0, 0, 0)")
        print(f"  - 胴体高さ: {self.body_height:.3f}m")
        print(f"  - 脚伸展率: {self.leg_extension_ratio*100:.0f}%")
        print(f"  - 接地時足先: Z≈0")

    def camber_trajectory(self, phase):
        """
        滑らかな楕円弧軌道生成（X-Z平面）
        phase: 0.0-1.0 (1歩行周期)
        returns: (x, z) 相対座標 (j13基準、股関節から見た足先の目標位置)
        
        座標系:
        - X軸: 前後方向（正が前方）
        - Z軸: 上下方向（正が上方、負が下方＝重力方向）
        
        軌道の特徴:
        - phase 0.0-0.5: 接地相（地面を蹴る、後ろから前へ）
        - phase 0.5-1.0: 遊脚相（持ち上げて戻る、前から後ろへ）
        - 接地/離地・着地時に滑らかに接続
        
        遊脚相は半楕円軌道で滑らかに持ち上げ・着地
        """
        half_step = self.step_length / 2
        
        if phase < 0.5:
            # 接地相: 地面を直線に蹴る (後ろから前へ)
            # phase 0.0 -> 0.5 で x: -half_step -> +half_step
            progress = phase / 0.5  # 0.0 -> 1.0
            x = half_step * (2 * progress - 1)  # -half_step -> +half_step
            z = 0  # 地面に接地
        else:
            # 遊脚相: 半楕円軌道で持ち上げて戻る (前から後ろへ)
            # phase 0.5 -> 1.0 で theta: 0 -> π
            progress = (phase - 0.5) / 0.5  # 0.0 -> 1.0
            theta = np.pi * progress  # 0 -> π
            
            # 楕円軌道: x = a*cos(θ), z = b*sin(θ)
            # θ=0 で (x=+half_step, z=0)、θ=π で (x=-half_step, z=0)
            # θ=π/2 で (x=0, z=step_height) が最高点
            x = half_step * np.cos(theta)   # +half_step -> -half_step
            z = self.step_height * np.sin(theta)  # 0 -> step_height -> 0
        
        return x, z
    
    def inverse_kinematics_2link(self, target_x, target_z, l1, l2):
        """
        2リンクの逆運動学 (股関節ピッチ, 膝ピッチ) - X-Z平面
        【逆関節（鳥脚型）用】: 膝が前方に曲がる
        
        初期姿勢（ピッチ角=0）: 脚はまっすぐ下に垂直
        - 大腿部: (0, 0) -> (0, -l1)
        - 下腿部: (0, -l1) -> (0, -l1-l2)
        
        逆関節の特徴:
        - 膝は前方に突出する（膝ピッチ角が負）
        - 大腿部は後傾、下腿部は前傾
        
        Args:
            target_x: 股関節ピッチ軸基準の目標足首X座標 [m]
            target_z: 股関節ピッチ軸基準の目標足首Z座標 [m]
            l1: 大腿部長さ (thigh) [m]
            l2: 下腿部長さ (shank) [m]
            
        Returns:
            (hip_pitch_angle, knee_pitch_angle) [degrees]
            - 左脚: j13, j14 / 右脚: j23, j24
            - 股関節ピッチ: 正=前方への回転、負=後方への回転
            - 膝ピッチ: 負=前方へ曲げる（逆関節）
        """
        
        # 目標までの距離
        d = np.sqrt(target_x**2 + target_z**2)
        
        # 到達可能性チェック
        max_reach = l1 + l2 - 0.001
        min_reach = abs(l1 - l2) + 0.001
        
        if d > max_reach:
            # 到達できない場合はスケール
            scale = max_reach / d
            target_x *= scale
            target_z *= scale
            d = max_reach
        elif d < min_reach:
            d = min_reach
            
        # 余弦定理で膝の角度を求める
        cos_knee = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        
        # 膝の内角（180度=伸びきり）
        knee_inner_angle = np.arccos(cos_knee)
        
        # 逆関節: 膝が前方に曲がる = 膝ピッチ角は負
        # knee_pitch = -(π - knee_inner_angle)
        j14 = -(np.pi - knee_inner_angle)
        
        # j13を求める
        # 目標への角度（垂直下向き=0を基準）
        theta_target = np.arctan2(target_x, -target_z)
        
        # 膝が前方に曲がった時の大腿部の補正角度
        # 逆関節では膝角度が負なので、補正も逆になる
        theta_correction = np.arctan2(l2 * np.sin(-j14), l1 + l2 * np.cos(-j14))
        
        # 股関節ピッチ: 後傾させる（逆関節の姿勢）
        j13 = theta_target + theta_correction
        
        return np.degrees(j13), np.degrees(j14)
    
    def calculate_foot_angle(self, hip_pitch_deg, knee_pitch_deg):
        """
        足部が地面に対して適切な角度を保つための足首ピッチ角度計算
        【逆関節用】: 足部は後方に伸びる（かかとで接地）
        
        Args:
            hip_pitch_deg: 股関節ピッチ角度 [degrees] (左脚:j13, 右脚:j23)
            knee_pitch_deg: 膝ピッチ角度 [degrees] (左脚:j14, 右脚:j24)
            
        Returns:
            足首ピッチ角度 [degrees] (左脚:j15, 右脚:j25)
        
        逆関節の足部:
        - 足首から後方（-X方向）に伸びる
        - 足首角度0度で足部は下腿と同一直線上（後方へ）
        - 接地のため、足部先端が地面に向くよう調整
        """
        # 逆関節: 下腿部の累積角度を打ち消して足部を地面に向ける
        # 足部が後方に伸びるため、90度のオフセットを追加
        total_leg_angle = hip_pitch_deg + knee_pitch_deg
        ankle_pitch = -total_leg_angle + 90  # 足部を後方水平に
        return ankle_pitch
    
    def generate_walking_pattern(self, t):
        """
        歩行パターン生成
        t: 時刻 [s]
        returns: (left_angles, right_angles) それぞれ5自由度
        """
        # 歩行周期での位相
        phase = (t * self.step_frequency) % 1.0
        
        # 左右で半周期ずれた位相
        left_phase = phase
        right_phase = (phase + 0.5) % 1.0
        
        # リンク長
        l1 = self.link_lengths['thigh']
        l2 = self.link_lengths['shank']
        
        # --- 左脚 ---
        # 直線往復軌道で目標位置生成（股関節ピッチ軸基準の相対座標）
        rel_x, rel_z = self.camber_trajectory(left_phase)
        
        # 基準足首位置: 脚全長×伸展率（自然な屈身状態）
        base_leg_height = (l1 + l2) * self.leg_extension_ratio
        target_left_x = 0 + rel_x
        target_left_z = -base_leg_height + rel_z
        
        # 2リンク逆運動学
        left_j13, left_j14 = self.inverse_kinematics_2link(
            target_left_x, target_left_z, l1, l2
        )
        
        # 足首角度（足裏を地面と平行に）
        left_j15 = self.calculate_foot_angle(left_j13, left_j14)
        
        left_angles = [
            0,           # j11: 腰ヨー (hip_yaw)
            0,           # j12: 股関節ロール (hip_roll)
            left_j13,    # j13: 股関節ピッチ (hip_pitch)
            left_j14,    # j14: 膝ピッチ (knee_pitch)
            left_j15     # j15: 足首ピッチ (ankle_pitch)
        ]
        
        # --- 右脚 ---
        rel_x, rel_z = self.camber_trajectory(right_phase)
        
        target_right_x = 0 + rel_x
        target_right_z = -base_leg_height + rel_z
        
        right_j23, right_j24 = self.inverse_kinematics_2link(
            target_right_x, target_right_z, l1, l2
        )
        
        right_j25 = self.calculate_foot_angle(right_j23, right_j24)
        
        right_angles = [
            0,            # j21: 腰ヨー (hip_yaw)
            0,            # j22: 股関節ロール (hip_roll)
            right_j23,    # j23: 股関節ピッチ (hip_pitch)
            right_j24,    # j24: 膝ピッチ (knee_pitch)
            right_j25     # j25: 足首ピッチ (ankle_pitch)
        ]
        
        return left_angles, right_angles
    
    def forward_kinematics_leg(self, angles, is_left=True):
        """
        順運動学（描画用）
        
        Args:
            angles: [yaw, roll, hip_pitch, knee_pitch, ankle_pitch] (degrees)
                    左脚: [j11, j12, j13, j14, j15]
                    右脚: [j21, j22, j23, j24, j25]
            is_left: True=左脚, False=右脚
            
        Returns:
            関節位置のリスト [[x,y,z], ...]
        
        座標系:
        - X軸: 前後方向（正が前、負が後）
        - Y軸: 左右方向（正が右、負が左）
        - Z軸: 上下方向（正が上、負が下）
        """
        yaw, roll, hip_pitch, knee_pitch, ankle_pitch = np.radians(angles)
        
        L = self.link_lengths
        side = 1 if is_left else -1
        
        positions = []
        
        # 0: 股関節中心（胴体中心から左右に分岐）
        hip_base = np.array([0, side * L['hip_width']/2, self.body_height])
        positions.append(hip_base)
        
        # 1: 腰ヨー軸後の位置 (j11/j21)
        yaw_offset = np.array([
            L['hip_yaw'] * np.sin(yaw),
            L['hip_yaw'] * np.cos(yaw) * side,
            0
        ])
        hip_yaw_end_pos = hip_base + yaw_offset
        positions.append(hip_yaw_end_pos)
        
        # 2: 股関節ロール軸後の位置 (j12/j22)
        # ロール軸リンクは下方向に延びる
        roll_link_vec = np.array([0, 0, -L['hip_roll']])
        hip_roll_end_pos = hip_yaw_end_pos + roll_link_vec
        positions.append(hip_roll_end_pos)
        
        # 3: 膝関節位置 (j14/j24の根元)
        # 股関節ピッチ(j13/j23)回転後の大腿部先端
        # 初期状態で下向き: (0, 0, -l1)
        thigh_vec = np.array([
            L['thigh'] * np.sin(hip_pitch),
            0,
            -L['thigh'] * np.cos(hip_pitch)
        ])
        
        # ロール回転適用（X軸周りの回転）
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        roll_matrix = np.array([
            [1, 0, 0],
            [0, c_roll, -s_roll],
            [0, s_roll, c_roll]
        ])
        thigh_vec = roll_matrix @ thigh_vec
        knee_pos = hip_roll_end_pos + thigh_vec
        positions.append(knee_pos)
        
        # 4: 足首位置 (j15/j25の根元)
        # 膝ピッチ(j14/j24)回転後の下腿部先端
        # 逆関節: 膝角度が負（前方に曲がる）
        total_pitch = hip_pitch + knee_pitch
        shank_vec = np.array([
            L['shank'] * np.sin(total_pitch),
            0,
            -L['shank'] * np.cos(total_pitch)
        ])
        shank_vec = roll_matrix @ shank_vec
        ankle_pos = knee_pos + shank_vec
        positions.append(ankle_pos)
        
        # 5: 足先位置
        # 逆関節の足部: 後方（-X方向）に伸びる
        # ankle_pitch=0 で下腿と同一直線、ankle_pitch=90で後方水平
        total_pitch_ankle = total_pitch + ankle_pitch
        # 足部は後方に伸びるため、-X方向成分を持つ
        foot_vec = np.array([
            -L['foot'] * np.cos(total_pitch_ankle),  # 後方（-X）成分
            0,
            -L['foot'] * np.sin(total_pitch_ankle)   # 下方成分
        ])
        foot_vec = roll_matrix @ foot_vec
        foot_pos = ankle_pos + foot_vec
        positions.append(foot_pos)
        
        return positions
    
    def run(self):
        """50Hzメインループ"""
        freq = 50
        dt = 1.0 / freq
        t = 0
        
        print("制御プロセス開始 (50Hz)")
        print("直線往復軌道パターン実行中 (X-Z平面)")
        print(f"- ストライド: {self.step_length}m (X方向)")
        print(f"- 足上げ高さ: {self.step_height}m (Z方向)")
        print(f"- 歩行周波数: {self.step_frequency}Hz")
        print(f"座標系: X=前後, Y=左右, Z=上下")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 歩行パターン生成
                left_angles, right_angles = self.generate_walking_pattern(t)
                
                # 2. 順運動学
                left_positions = self.forward_kinematics_leg(left_angles, is_left=True)
                right_positions = self.forward_kinematics_leg(right_angles, is_left=False)
                
                # 3. データパッケージ
                data = {
                    'timestamp': time.time(),
                    'cycle': t,
                    'joint_angles': {
                        'j11': left_angles[0], 'j12': left_angles[1], 'j13': left_angles[2], 
                        'j14': left_angles[3], 'j15': left_angles[4],
                        'j21': right_angles[0], 'j22': right_angles[1], 'j23': right_angles[2],
                        'j24': right_angles[3], 'j25': right_angles[4]
                    },
                    'left_positions': np.array(left_positions).tolist(),
                    'right_positions': np.array(right_positions).tolist()
                }
                
                # 4. 送信
                self.socket.send_pyobj(data)
                
                # デバッグ: 送信確認（最初の10回と、その後は1秒ごと）
                if not hasattr(self, '_send_count'):
                    self._send_count = 0
                self._send_count += 1
                if self._send_count <= 10 or abs(t % 1.0) < dt:
                    print(f"[Send #{self._send_count}] t={t:.2f}s, cycle={data['cycle']:.2f}, L_j13={left_angles[2]:+6.1f}°")
                
                t += dt
                
                # 周期維持
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n制御プロセス終了")
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    controller = BipedController()
    controller.run()