#!/usr/bin/env python3
"""
Sinusoidal Motion Publisher for RobStride Motor

50Hzで動作するpublisherノード
±90度（±π/2 rad）を1秒に2回往復する正弦波モーションを生成
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class SinusoidalMotionPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_motion_publisher')
        
        # パラメータ宣言
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('amplitude', math.pi / 2)  # ±90度 = ±π/2 rad
        self.declare_parameter('frequency', 2.0)  # 1秒に2回往復 = 2 Hz
        self.declare_parameter('topic_name', '/forward_position_controller/commands')
        
        # パラメータ取得
        self.publish_rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        topic_name = self.get_parameter('topic_name').value
        
        # Publisher作成 (SystemDefaultsQoSに合わせる - depth=10, RELIABLE, VOLATILE)
        # Forward Command Controllerと同じQoS設定を使用
        self.publisher = self.create_publisher(
            Float64MultiArray,
            topic_name,
            10  # SystemDefaultsQoS equivalent
        )
        
        # タイマー作成 (50Hz)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 開始時刻記録
        self.start_time = self.get_clock().now()
        
        # 最初に0位置をpublishして初期化
        initial_msg = Float64MultiArray()
        initial_msg.data = [0.0]
        self.publisher.publish(initial_msg)
        
        self.get_logger().info(
            f'Sinusoidal Motion Publisher started:\n'
            f'  Publish Rate: {self.publish_rate} Hz\n'
            f'  Amplitude: ±{math.degrees(self.amplitude):.1f} deg (±{self.amplitude:.3f} rad)\n'
            f'  Frequency: {self.frequency} Hz (period: {1.0/self.frequency:.3f} sec)\n'
            f'  Topic: {topic_name}'
        )
    
    def timer_callback(self):
        """
        50Hzで呼ばれるコールバック
        正弦波の目標位置を計算してpublish
        """
        # 経過時間計算
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9
        
        # 最初の1秒間は0を送信してコントローラーを安定化
        if elapsed_sec < 1.0:
            position = 0.0
        else:
            # 正弦波生成: position = amplitude * sin(2π * frequency * (t-1.0))
            # 1秒後から開始するため、t-1.0で位相を調整（0度からスタート）
            position = self.amplitude * math.sin(2 * math.pi * self.frequency * (elapsed_sec - 1.0))
        
        # メッセージ作成
        msg = Float64MultiArray()
        msg.data = [position]
        
        # Publish
        self.publisher.publish(msg)
        
        # ログ出力 (2秒に1回 = 50Hz * 2s = 100回に1回)
        if int(elapsed_sec * self.publish_rate) % 100 == 0:
            self.get_logger().info(
                f't={elapsed_sec:.2f}s | target_pos={position:.3f} rad ({math.degrees(position):.1f} deg)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalMotionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
