#!/usr/bin/env python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped, Quaternion
from std_msgs.msg import String
from robot_interfaces.srv import Scheduler, RandomEndeffector, ControllerData
import numpy as np

#Inverse Kinematics
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Create controller Server
        self.controller_server = self.create_service(ControllerData, "controller_server", self.controller_server_callback)
        self.controller_state = "IDLE"

        # Create robot state subscriber
        self.current_state = "IDLE"
        self.create_subscription(String, "/current_state", self.current_state_callback, 10)

        # Create TF subscriber to pub topic /end_effector
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "end_effector"
        self.source_frame = "link_0"



        self.get_logger().info("controller_node has been started.")

    def controller_server_callback(self, request: ControllerData.Request, response: ControllerData.Response):
        self.get_logger().info(f"Request to change controller state from {self.controller_state} to: {request.mode.data}")
        self.controller_state = request.mode.data

        response.inprogress = True
        return response

    def current_state_callback(self, msg: String):
        self.current_state = msg.data

    def get_transform(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'link_0',
                'end_effector',
                now)
            position = transform.transform.translation
            orientation = transform.transform.rotation
            x = position.x
            y = position.y
            z = position.z


            self.get_logger().info(
                f"End Effector Position: {position.x}, {position.y}, {position.z} "
            )
            # self.get_logger().info(f"End Effector Orientation: {orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}")

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")

    def timer_callback(self):
        self.get_transform()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
