#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from math import sqrt


class LocalizationVisualizer(Node):
    def __init__(self):
        super().__init__('localization_visualizer')
        
        # Subscriber
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/visual_localization/pose',
            self.pose_callback,
            10
        )
        
        # Publisher
        self.marker_pub = self.create_publisher(
            Marker,
            '/localization_pose_marker',
            10
        )
        
        self.get_logger().info('Localization Visualizer Node started')
    
    def pose_callback(self, msg):
        # Extract pose information
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.get_logger().info(
            f'Received pose - '
            f'Position: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}], '
            f'Orientation: [{ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f}]'
        )
        
        # Create and publish arrow marker
        self.publish_arrow_marker(msg)
        # Create and publish covariance ellipsoid marker
        self.publish_covariance_marker(msg)
    
    def publish_arrow_marker(self, pose_msg):
        """Publish arrow marker to visualize pose direction"""
        marker = Marker()
        marker.header = pose_msg.header
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Scale: [length, width, height]
        marker.scale.x = 0.5   # arrow length
        marker.scale.y = 0.1   # arrow width
        marker.scale.z = 0.1   # arrow height
        
        # Color: Green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Position and orientation
        marker.pose = pose_msg.pose.pose
        
        self.marker_pub.publish(marker)
    
    def publish_covariance_marker(self, pose_msg):
        """Publish covariance ellipsoid marker"""
        marker = Marker()
        marker.header = pose_msg.header
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Extract position covariance (3x3 block)
        cov = pose_msg.pose.covariance
        # Covariance is [36] array: position (0-2,0-2), rotation (3-5,3-5)
        pos_cov_xx = cov[0]   # x variance
        pos_cov_yy = cov[7]   # y variance
        pos_cov_zz = cov[14]  # z variance
        
        # Scale based on covariance (1-sigma)
        marker.scale.x = 2.0 * sqrt(max(pos_cov_xx, 0.001))
        marker.scale.y = 2.0 * sqrt(max(pos_cov_yy, 0.001))
        marker.scale.z = 2.0 * sqrt(max(pos_cov_zz, 0.001))
        
        # Color: Semi-transparent blue
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.3
        
        # Position only (no rotation for covariance ellipsoid)
        marker.pose.position = pose_msg.pose.pose.position
        marker.pose.orientation.w = 1.0
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()