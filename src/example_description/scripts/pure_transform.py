#!/usr/bin/python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped, Quaternion
from std_msgs.msg import String
from robot_interfaces.srv import Scheduler, RandomEndeffector, ControllerData
import numpy as np


class PureTransformNode(Node):
    def __init__(self):
        super().__init__("pure_transform_node")

        # Set Callback Frequency
        self.declare_parameter("frequency", 10.0)
        self.frequency = (self.get_parameter("frequency").get_parameter_value().double_value)
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Create Service Client for RandomEndeffector
        self.random_client = self.create_client(RandomEndeffector, "random_server")
        while not self.random_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'random_server' service...")

        # Create JointState Publisher
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.joint_state = JointState()
        self.joint_state.header.frame_id = ""  
        self.joint_state.name = ["joint_1", "joint_2", "joint_3"] 
        self.joint_state.position = [0.0, 0.0, 0.0]

        self.get_logger().info("pure_transform_node has been started.")

    def req_random(self):
        self.get_logger().info("Request random node")
        state_request = RandomEndeffector.Request()
        state_request.mode.data = "AUTO"
        future = self.random_client.call_async(state_request)
        future.add_done_callback(self.callback_req_random)

    def callback_req_random(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Received Response - x: {response.position.x}, y: {response.position.y}, z: {response.position.z}"
            )

            positions = [
                response.position.x,  # joint_1 position
                response.position.y,  # joint_2 position
                response.position.z,  # joint_3 position
            ]
            self.publish_joint_state(positions)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def publish_joint_state(self, positions):

        if len(positions) != len(self.joint_state.name):
            self.get_logger().error(
                f"Number of positions ({len(positions)}) does not match number of joints ({len(self.joint_state.name)})."
            )
            return

        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Update joint positions
        self.joint_state.position = positions

        self.joint_state_publisher.publish(self.joint_state)
        self.get_logger().info("Published JointState message.")

    def timer_callback(self):
        self.req_random()
        


def main(args=None):
    rclpy.init(args=args)
    node = PureTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
