#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus

class HWStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")

        # Define the parameters
        self.declare_parameter(name="motors_ready", value=True)
        self.declare_parameter(name="temperature", value=34)

        self.publisher = self.create_publisher(msg_type=HardwareStatus, # msg type
                                               topic="hw_status", # Topic name
                                               qos_profile=10) # Queue size

        self.timer = self.create_timer(timer_period_sec=2, callback=self.publish_status)
        self.get_logger().info("HW Status publisher has been started")

    def publish_status(self):
        # Retrieve the parameters 
        msg = HardwareStatus()
        msg.temperature = self.get_parameter(name="temperature").value
        msg.are_motors_ready = self.get_parameter(name="motors_ready").value
        
        msg.debug = "Everything is fine!"
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = HWStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()