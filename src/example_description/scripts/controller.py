#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped, Quaternion
from std_msgs.msg import String
from robot_interfaces.srv import Scheduler, RandomEndeffector, ControllerData
import numpy as np

# Inverse Kinematics
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3

from sensor_msgs.msg import JointState  # Import JointState message


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Create controller Server
        self.controller_server = self.create_service(
            ControllerData, "controller_server", self.controller_server_callback
        )
        self.controller_state = "IDLE"

        # Create robot state subscriber
        self.current_state = "IDLE"
        self.create_subscription(
            String, "/current_state", self.current_state_callback, 10
        )

        # Create TF subscriber to pub topic /end_effector
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "end_effector"
        self.source_frame = "link_0"

        # Robot parameters
        self.kp = 1.0
        self.q = np.array([0.0, 0.0, 0.0])
        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2
        self.ik_setpoint = [0, 0, 0]
        self.random_setpoint = [0, 0, 0]

        # Initialize JointState publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self.joint_state = JointState()
        self.joint_state.header.frame_id = ""
        self.joint_state.name = ["joint_1", "joint_2", "joint_3"]
        self.joint_state.position = [0.0, 0.0, 0.0]

        self.get_logger().info("controller_node has been started.")

    def controller_server_callback(
        self, request: ControllerData.Request, response: ControllerData.Response
    ):
        self.get_logger().info(
            f"Request to change controller state from {self.controller_state} to: {request.mode.data}"
        )
        self.controller_state = request.mode.data
        if self.controller_state == "AUTO":
            self.random_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]

        response.inprogress = True
        return response

    def current_state_callback(self, msg: String):
        self.current_state = msg.data

    def publish_joint_state(self, positions):
        if len(positions) != len(self.joint_state.name):
            self.get_logger().error(
                f"Number of positions ({len(positions)}) does not match number of joints ({len(self.joint_state.name)})."
            )
            return

        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Update joint positions
        self.joint_state.position = positions.tolist()  # Ensure it's a list

        self.joint_state_publisher.publish(self.joint_state)
        self.get_logger().info("Published JointState message.")

    def control_to_pos(self, p_set):
        try:
            # Robot definition
            robot = rtb.DHRobot(
                [
                    rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                    rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                    rtb.RevoluteMDH(alpha=0.0, a=0.25, d=0.0, offset=0.0),
                ],
                tool=SE3.Tx(0.28),
                name="RRR_Robot",
            )

            p_setpoint = np.array(p_set)
            p_now = self.get_transform()
            if p_now is None:
                self.get_logger().error(
                    "Current end-effector position could not be retrieved."
                )
                return False

            error = p_setpoint - p_now
            # Compute desired end-effector velocity (p_dot)
            p_dot = self.kp * error
            # Compute Jacobian
            J = robot.jacob0(self.q)
            J_pos = J[0:3, :]  # Position part of the Jacobian
            # Compute joint velocities (q_dot)
            q_dot = np.linalg.pinv(J_pos) @ p_dot

            # Update joint angles
            self.q = self.q + q_dot * (1.0 / self.frequency)
            self.get_logger().info(f"q : {self.q}")

            # Publish joint states
            self.publish_joint_state(self.q)

            if np.linalg.norm(error) <= 0.001:
                self.get_logger().info("Target position reached.")
                self.controller_state = "IDLE"
                return False
            else:
                return True

        except Exception as e:
            self.get_logger().error(f"Failed in control_to_pos: {e}")
            return False

    def get_transform(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform("link_0", "end_effector", now)
            position = transform.transform.translation
            orientation = transform.transform.rotation
            x = position.x
            y = position.y
            z = position.z

            self.get_logger().info(f"End Effector Position: {x}, {y}, {z}")
            # self.get_logger().info(f"End Effector Orientation: {orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}")

            return np.array([x, y, z])  # Return the position as an array

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    def inverse_kinematic(self, x, y, z):
        distance_squared = x**2 + y**2 + (z - 0.2) ** 2
        if self.r_min**2 <= distance_squared <= self.r_max**2:

            robot = rtb.DHRobot(
                [
                    rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                    rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                    rtb.RevoluteMDH(alpha=0, a=0.25, d=0.0, offset=0.0),
                ],
                tool=SE3.Tx(0.28),
                name="RRR_Robot",
            )

            T_Position = SE3(x, y, z)
            ik_solution = robot.ikine_LM(
                T_Position, mask=[1, 1, 1, 0, 0, 0], joint_limits=False, q0=[0, 0, 0]
            )

            if ik_solution.success:
                q_sol_ik_LM = ik_solution.q  # Extract joint angles
                return q_sol_ik_LM
            else:
                self.get_logger().error("Inverse kinematics failed to find a solution.")
                return None
        else:
            self.get_logger().error("Target position is out of reach.")
            return None

    def timer_callback(self):
        try:
            if self.controller_state == "AUTO":
                continue_control = self.control_to_pos(self.random_setpoint)
                if not continue_control:
                    self.get_logger().info("Switching to IDLE state.")
                    self.controller_state = "IDLE"
        except Exception as e:
            self.get_logger().error(f"Failed in timer_callback: {e}")
            return None


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


if __name__ == "__main__":
    main()
