#!/usr/bin/python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import String
from robot_interfaces.srv import Scheduler,RandomEndeffector
import random

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')
        # Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.create_timer(1 / self.frequency, self.timer_callback)

        self.target_pub = self.create_publisher(PoseStamped, 'target', 10)

        # Set Subscribe Current State
        self.create_subscription(String, "/current_state", self.current_state_callback, 10)

        # Create Random Server for calculate inverse kinematics
        self.random_server = self.create_service(RandomEndeffector, "random_server", self.random_server_callback)
        self.r_max = 0.7294987304567765
        self.r_min = 0.03152356441013133
        self.l = 0.2

        self.current_state = "IDLE"
        self.get_logger().info("random_node has been started.")

    def random_server_callback(self, request:RandomEndeffector, response:RandomEndeffector):

        self.get_logger().info(f"Request {str(request.mode.data)} mode")
        if request.mode.data == "AUTO":

            while True:
                x = random.uniform(-self.r_max, self.r_max)
                y = random.uniform(-self.r_max, self.r_max)
                z = random.uniform(-self.r_max, self.r_max)

                size_squared = x**2 + y**2 + (z - self.l)**2

                if self.r_min**2 < size_squared < self.r_max**2:

                    self.goal = [x, y, z]

                    msg = PoseStamped()
                    msg.pose.position.x = x
                    msg.pose.position.y = y
                    msg.pose.position.z = z

                    response.position.x = x
                    response.position.y = y
                    response.position.z = z
                    response.inprogress = True

                    # response.message = f"Target set at: {x, y, z}"
                    self.target_pub.publish(msg)
                    break
        return response

    def current_state_callback(self, msg:String):
        self.current_state = msg.data      

    def timer_callback(self):
        # print(self.target)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
