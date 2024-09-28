#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from controller_manager_msgs.srv import LoadController,ConfigureController
from robot_interfaces.srv import Scheduler
from std_msgs.msg import String

class TeleopTwistKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_key_node')
        
        #Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.create_timer(1 / self.frequency, self.timer_callback)
        
        #Set Subscribe Current State
        self.create_subscription(String, "/current_state", self.current_state_callback, 10)
        
        #Set Robot Parameter
        self.current_state = "IDLE"
        self.get_logger().info("teleop_twist_key_node has been started.")
        
    def current_state_callback(self, msg:String):
        self.current_state = msg.data        

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_key(self,key):
        send_key_request = ConfigureController.Request()
        send_key_request.name = str(key)
        self.configure_controller_client.call_async(send_key_request)
        
    def timer_callback(self):
        
        key = self.getKey()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistKeyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
