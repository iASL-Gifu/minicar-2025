#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import time

class BackControlNode(Node):
    def __init__(self):
        super().__init__('back_control_node')
        
        # パラメータ宣言（バック制御に適した初期値）
        self.declare_parameter('throttle_value', -0.3)  # バック方向
        self.declare_parameter('steering_value', 0.0)   # 直進
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('enable_on_start', False)
        self.declare_parameter('auto_stop_duration', 3.0)  # バック制御は短時間で停止
        
        # パラメータ取得
        self.throttle_value = self.get_parameter('throttle_value').value
        self.steering_value = self.get_parameter('steering_value').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.enable_on_start = self.get_parameter('enable_on_start').value
        self.auto_stop_duration = self.get_parameter('auto_stop_duration').value
        
        # 状態管理
        self.is_enabled = self.enable_on_start
        self.start_time = time.time()
        
        # パブリッシャー
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, 
            '/ackermann_cmd', 
            10
        )
        
        # サブスクライバー（有効/無効切り替え用）
        self.enable_sub = self.create_subscription(
            Bool,
            '/back_control/enable',
            self.enable_callback,
            10
        )
        
        # タイマー
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 初期化完了メッセージ
        self.get_logger().info('Back Control Node initialized')
        self.get_logger().info(f'  Throttle: {self.throttle_value}')
        self.get_logger().info(f'  Steering: {self.steering_value}')
        self.get_logger().info(f'  Frequency: {self.publish_frequency} Hz')
        self.get_logger().info(f'  Enabled: {self.is_enabled}')
        self.get_logger().info(f'  Auto stop after: {self.auto_stop_duration} seconds')
        self.get_logger().info('')
        self.get_logger().info('Control commands:')
        self.get_logger().info('  Enable:  ros2 topic pub /back_control/enable std_msgs/Bool "data: true"')
        self.get_logger().info('  Disable: ros2 topic pub /back_control/enable std_msgs/Bool "data: false"')
    
    def enable_callback(self, msg):
        """有効/無効切り替えコールバック"""
        old_state = self.is_enabled
        self.is_enabled = msg.data
        
        if old_state != self.is_enabled:
            if self.is_enabled:
                self.get_logger().info('🟢 Back control ENABLED')
                self.start_time = time.time()  # 開始時間をリセット
            else:
                self.get_logger().info('🔴 Back control DISABLED')
    
    def timer_callback(self):
        """定期実行コールバック"""
        # メッセージ作成
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 自動停止チェック
        elapsed_time = time.time() - self.start_time
        if self.is_enabled and elapsed_time > self.auto_stop_duration:
            self.get_logger().info(f'⏰ Auto stop after {self.auto_stop_duration} seconds')
            self.is_enabled = False
        
        # 制御値設定
        if self.is_enabled:
            msg.drive.speed = self.throttle_value
            msg.drive.steering_angle = self.steering_value
        else:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
        
        # パブリッシュ
        self.cmd_pub.publish(msg)
        
        # ログ出力（1秒に1回）
        if int(elapsed_time) % int(self.publish_frequency) == 0:
            status = "🟢 ACTIVE" if self.is_enabled else "🔴 STOPPED"
            remaining = max(0, self.auto_stop_duration - elapsed_time) if self.is_enabled else 0
            self.get_logger().info(
                f'{status} | Speed: {msg.drive.speed:.2f} | Steer: {msg.drive.steering_angle:.2f} | '
                f'Time: {elapsed_time:.1f}s | Remaining: {remaining:.1f}s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = BackControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()