#!/usr/bin/env python3

#slam_pose_publisher.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SlamPosePublisher(Node):
    def __init__(self):
        super().__init__('slam_pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publish SLAM's exact position as an Odometry message
        self.pose_pub = self.create_publisher(Odometry, '/slam_pose', 10)
        self.timer = self.create_timer(0.1, self.on_timer) # 10Hz updates

        self.get_logger().info('Publishing SLAM-only position to /slam_pose')

    def on_timer(self):
        try:
            # Ask TF for SLAM's calculation of where the robot is on the map
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation
        
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = SlamPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()