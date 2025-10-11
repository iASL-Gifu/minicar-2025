import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from jetracer.nvidia_racecar import NvidiaRacecar
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.time import Time

class JetRacerDriver(Node):
    def __init__(self):
        super().__init__('jetracer_driver')

        # パラメータ宣言
        self.declare_parameter('throttle_inversion', False)
        self.declare_parameter('steering_inversion', False)
        self.declare_parameter('steering_offset', 0.0)
        self.declare_parameter('throttle_offset', 0.0)
        self.declare_parameter('offset_step', 0.01)
        self.declare_parameter('throttle_gain', 1.0)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('max_command_age', 0.5)

        self.car = NvidiaRacecar()
        self.last_cmd_time = self.get_clock().now()
        self.car.throttle = 0.0
        self.car.steering = 0.0

        qos = QoSProfile(depth=10)

        self.create_subscription(
            AckermannDriveStamped,
            '/cmd_drive',  # AckermannDriveStampedを受け取るトピック名
            self._cmd_cb,
            qos
        )

        # オフセット調整トピック（変更なし）
        self.create_subscription(Bool, '/steer_offset_inc', self._steer_offset_inc_cb, qos)
        self.create_subscription(Bool, '/steer_offset_dec', self._steer_offset_dec_cb, qos)
        self.create_subscription(Bool, '/speed_offset_inc', self._speed_offset_inc_cb, qos)
        self.create_subscription(Bool, '/speed_offset_dec', self._speed_offset_dec_cb, qos)

        self.create_timer(0.1, self._watchdog)
        self.get_logger().info('JetRacer driver started, waiting for /cmd_drive')

    def _get_param(self, name: str, param_type: Parameter.Type = Parameter.Type.DOUBLE) -> any:
        return self.get_parameter(name).get_parameter_value()

    def _set_param(self, name: str, value: float):
        self.set_parameters([Parameter(name, Parameter.Type.DOUBLE, value)])
        self.get_logger().info(f'{name} updated to: {value:.3f}')

    # コールバックの型ヒントを更新
    def _cmd_cb(self, msg: AckermannDriveStamped):
        # --- 鮮度チェック（最も重要な追加機能） ---
        now = self.get_clock().now()
        msg_stamp = Time.from_msg(msg.header.stamp)
        age = (now - msg_stamp).nanoseconds / 1e9  
        max_age = self._get_param('max_command_age').double_value

        if age > max_age:
            self.get_logger().warn(
                f'Command is too old ({age:.3f}s > {max_age:.3f}s), ignoring.')
            # 古いコマンドは実行せずに即座にリターンする
            return
        
        # パラメータを取得
        steering_offset = self._get_param('steering_offset').double_value
        throttle_offset = self._get_param('throttle_offset').double_value
        throttle_inversion = self._get_param('throttle_inversion').bool_value
        steering_inversion = self._get_param('steering_inversion').bool_value
        throttle_gain = self._get_param('throttle_gain').double_value
        steering_gain = self._get_param('steering_gain').double_value
        
        # 制御値は .drive フィールドから取得
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle

        # スロットル計算
        throttle = (speed * throttle_gain) + throttle_offset
        if throttle_inversion:
            throttle *= -1.0
        throttle = max(min(throttle, 1.0), -1.0)

        # ステアリング計算
        steering = (steering_angle * steering_gain) + steering_offset
        if steering_inversion:
            steering *= -1.0
        steering = max(min(steering, 1.0), -1.0)

        self.car.throttle = float(throttle)
        self.car.steering = float(steering)
        self.last_cmd_time = now # 有効なコマンドを受け取った時刻を更新

    def _watchdog(self):
        # 1秒以上コマンドが来ていなければ停止（二次的な安全装置として維持）
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9:
            self.car.throttle = 0.0
            self.car.steering = 0.0

    # --- 動的調整コールバック ---
    def _steer_offset_inc_cb(self, msg: Bool):
        if msg.data:
            step = self._get_param('offset_step').double_value
            current = self._get_param('steering_offset').double_value
            self._set_param('steering_offset', current + step)
    
    def _steer_offset_dec_cb(self, msg: Bool):
        if msg.data:
            step = self._get_param('offset_step').double_value
            current = self._get_param('steering_offset').double_value
            self._set_param('steering_offset', current - step)

    def _speed_offset_inc_cb(self, msg: Bool):
        if msg.data:
            step = self._get_param('offset_step').double_value
            current = self._get_param('throttle_offset').double_value
            self._set_param('throttle_offset', current + step)

    def _speed_offset_dec_cb(self, msg: Bool):
        if msg.data:
            step = self._get_param('offset_step').double_value
            current = self._get_param('throttle_offset').double_value
            self._set_param('throttle_offset', current - step)


def main():
    rclpy.init()
    node = JetRacerDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()