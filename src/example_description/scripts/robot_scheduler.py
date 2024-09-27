#!/usr/bin/python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Twist
from robot_interfaces.srv import Scheduler,RandomEndeffector,ControllerData
from std_msgs.msg import String

class RobotSchedulerNode(Node):
    def __init__(self):
        super().__init__('robot_scheduler_node')
        
        #Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.create_timer(1 / self.frequency, self.timer_callback)
        
        #Create Robot state Server and Publisher
        self.random_client = self.create_client(RandomEndeffector,"random_server")
        self.controller_client = self.create_client(Scheduler,"controller_server")
        self.robot_state_server = self.create_service(Scheduler, "robot_state_server", self.robot_state_server_callback)
        self.robot_state_pub = self.create_publisher(String, "current_state", 10)
        self.current_state = "IDLE"
        
        self.get_logger().info("robot_scheduler_node has been started.")
    
    def req_random(self):
        self.get_logger().info("Request random node")
        state_request = RandomEndeffector.Request()
        state_request.mode.data = "AUTO"
        future = self.random_client.call_async(state_request)
        future.add_done_callback(self.callback_req_random)
        
    def callback_req_random(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"Got x:{response.position.x} y:{response.position.y} z:{response.position.z}")
        except Exception as e:
            self.get_logger().info("Can't connect to Controller Server")
            
        
    def req_controller(self):
        
        pass
    
    
    def robot_state_server_callback(self, request:Scheduler, response:Scheduler):
        self.get_logger().info(f"Request To Change State from {str(self.current_state)} to : {str(request.state.data)}")
        self.current_state = str(request.state.data)
        
        if str(request.state.data) == "AUTO":
            self.req_random()
        
        response.inprogress = True
        return response
    
    def timer_callback(self):
        msg = String()
        msg.data = self.current_state
        self.robot_state_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = RobotSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
