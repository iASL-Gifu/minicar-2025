#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class FrameIdExchanger(Node):
    def __init__(self):
        super().__init__('frame_id_exchanger')
        
        # パラメータを宣言
        self.declare_parameter('config_file', '')
        self.declare_parameter('input_topic', '/camera/infra2/camera_info')
        self.declare_parameter('output_topic', '/right/camera_info_rect')
        self.declare_parameter('new_frame_id', 'camera_infra2_optical_frame')
        
        # パラメータを取得
        config_file = self.get_parameter('config_file').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.new_frame_id = self.get_parameter('new_frame_id').value
        
        # 設定ファイルから読み込む場合
        if config_file:
            self.load_config(config_file)
        
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'New frame_id: {self.new_frame_id}')
        
        # サブスクライバーとパブリッシャーを作成
        self.subscription = self.create_subscription(
            CameraInfo,
            self.input_topic,
            self.camera_info_callback,
            10)
        
        self.publisher = self.create_publisher(
            CameraInfo,
            self.output_topic,
            10)
    
    def load_config(self, config_file):
        """YAML設定ファイルから設定を読み込む"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'frame_id_exchanger' in config:
                cfg = config['frame_id_exchanger']
                self.input_topic = cfg.get('input_topic', self.input_topic)
                self.output_topic = cfg.get('output_topic', self.output_topic)
                self.new_frame_id = cfg.get('new_frame_id', self.new_frame_id)
                self.get_logger().info(f'Loaded config from {config_file}')
        except FileNotFoundError:
            self.get_logger().warn(f'Config file not found: {config_file}')
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}')
    
    def camera_info_callback(self, msg):
        """CameraInfo メッセージを受け取り、frame_id を変更してパブリッシュ"""
        msg.header.frame_id = self.new_frame_id
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameIdExchanger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()