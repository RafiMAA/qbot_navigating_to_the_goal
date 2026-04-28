#!/usr/bin/env python3

#qbot_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# HARDWARE UPDATE: Using standard Twist instead of TwistStamped
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry 

from interfaces.action import Navigation
from interfaces.msg import Position

class QbotControllerNode(Node):

    def __init__(self):
        super().__init__('qbot_controller')

        self.declare_parameter('linear_speed', 0.8)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        self.current_yaw = 0.0
        self.goal_done   = True
        self.goal_sent   = False
        self.mode        = 'idle'
        self._goal_handle = None

        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, '/slam_pose', self.update_yaw_from_odom, 10,
            callback_group=self._cb_group)

        self.create_subscription(
            Point, '/ui_goal', self.on_ui_goal, 10,
            callback_group=self._cb_group)

        # HARDWARE UPDATE: Publishing to the Kobuki's native velocity topic
        self.cmd_pub = self.create_publisher(
            Twist, '/commands/velocity', 10)

        self._action_client = ActionClient(
            self, Navigation, 'navigate',
            callback_group=self._cb_group)

        self.get_logger().info('Qbot Hardware Controller ready')

    def update_yaw_from_odom(self, msg):
        try:
            o = msg.pose.pose.orientation
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            pass

    def on_ui_goal(self, msg):
        if self.mode == 'navigating':
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self.mode = 'idle'
            self.goal_done = True

        x, y = msg.x, msg.y
        self.mode      = 'navigating'
        self.goal_done = False
        self.goal_sent = False
        self._send_goal(x, y)

    def _send_goal(self, x, y):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.mode = 'idle'
            return

        goal_msg = Navigation.Goal()
        position   = Position()
        position.x = float(x)
        position.y = float(y)
        goal_msg.end_position = position

        self.goal_sent = True
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.handle_navigation_feedback
        )
        future.add_done_callback(self.handle_navigation_goal_response)

    def handle_navigation_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.mode = 'idle'
                return
            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.handle_navigation_result)
        except Exception as e:
            self.mode = 'idle'

    def handle_navigation_result(self, future):
        self.goal_done = True
        self.mode      = 'idle'
        self._goal_handle = None
        
        # HARDWARE UPDATE: Stop using standard Twist
        stop = Twist()
        self.cmd_pub.publish(stop)

    def handle_navigation_feedback(self, feedback_msg):
        if self.goal_done or self.mode != 'navigating':
            return
        try:
            direction = feedback_msg.feedback.direction
            yaw_error = self.angle_diff(direction, self.current_yaw)

            # HARDWARE UPDATE: Command using standard Twist
            cmd = Twist()

            abs_yaw_error = abs(yaw_error)
            speed_factor = max(0.2, 1.0 - (abs_yaw_error / 1.57))
            
            cmd.linear.x = self.linear_speed * speed_factor
            cmd.angular.z = 1.0 * yaw_error
            
            self.cmd_pub.publish(cmd)

        except Exception as e:
            pass

    @staticmethod
    def angle_diff(desired, current):
        diff = desired - current
        return math.atan2(math.sin(diff), math.cos(diff))

def main(args=None):
    rclpy.init(args=args)
    node = QbotControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()