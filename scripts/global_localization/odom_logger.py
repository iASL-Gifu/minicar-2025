#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime

class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
        
        # データを蓄積するリスト
        self.path_data = []
        
        # BASE_PATHを環境変数から取得（デフォルトは./data）
        base_path = os.environ.get('BASE_PATH', '/workspaces/src/launch/localization_launch/path')
        
        # ディレクトリが存在しない場合は作成
        os.makedirs(base_path, exist_ok=True)
        
        # CSVファイル名（タイムスタンプ付き）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(base_path, f"odometry_path_{timestamp}.csv")
        
        # Odometryトピックをサブスクライブ
        # トピック名は環境に応じて変更してください
        self.subscription = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Odometry logger started')
        self.get_logger().info(f'Data will be saved to {self.csv_filename}')
        self.get_logger().info('Press Ctrl+C to stop and save data')
        
    def odom_callback(self, msg):
        """Odometryメッセージのコールバック"""
        # 位置情報を取得
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # タイムスタンプ（秒単位）
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # データを蓄積
        self.path_data.append({
            'timestamp': timestamp,
            'x': x,
            'y': y,
            'z': z
        })
        
        # 進捗表示（100点ごと）
        if len(self.path_data) % 100 == 0:
            self.get_logger().info(f'Collected {len(self.path_data)} data points')
    
    def save_to_csv(self):
        """データをCSVファイルに保存"""
        if not self.path_data:
            self.get_logger().warn('No data to save!')
            return
        
        with open(self.csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'x', 'y', 'z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for data_point in self.path_data:
                writer.writerow(data_point)
        
        self.get_logger().info(f'Data saved to {self.csv_filename}')
        self.get_logger().info(f'Total data points: {len(self.path_data)}')

def main(args=None):
    rclpy.init(args=args)
    
    logger = OdometryLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        logger.save_to_csv()
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()