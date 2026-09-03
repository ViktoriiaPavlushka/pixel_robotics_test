#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.safe_front = True
        self.safe_rear = True
        
        self.robot_half_width = 0.10
        self.lateral_margin = 0.05 

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.teleop_sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.teleop_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

    def scan_callback(self, msg):
        danger_front = False
        danger_rear = False

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if abs(y) <= (self.robot_half_width + self.lateral_margin):
     
                if 0 < x <= 0.5:
                    danger_front = True
                elif -0.5 <= x < 0:
                    danger_rear = True

        if danger_front and self.safe_front:
            self.get_logger().warn('FRONT collision predicted! Stopping.')
            self.force_stop()
        if danger_rear and self.safe_rear:
            self.get_logger().warn('REAR collision predicted! Stopping.')
            self.force_stop()

        self.safe_front = not danger_front
        self.safe_rear = not danger_rear

    def force_stop(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

    def teleop_callback(self, msg):
        safe_msg = TwistStamped()
        safe_msg.header = msg.header
        safe_msg.header.stamp = self.get_clock().now().to_msg()
        safe_msg.twist = msg.twist

        if safe_msg.twist.linear.x > 0.0 and not self.safe_front:
            safe_msg.twist.linear.x = 0.0
        elif safe_msg.twist.linear.x < 0.0 and not self.safe_rear:
            safe_msg.twist.linear.x = 0.0

        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
