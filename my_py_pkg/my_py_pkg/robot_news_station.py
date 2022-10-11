#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String # Rememeber to add the depenedency in package.xml

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.declare_parameter(name="robot_name", value="R2D2")
        self.declare_parameter(name="publish_period", value=1)

        self.robot_name = self.get_parameter("robot_name").value
        self.publisher = self.create_publisher(msg_type=String, # msg type
                                               topic="robot_news", # Topic name
                                               qos_profile=10) # Queue size

        self.timer = self.create_timer(
            timer_period_sec=self.get_parameter("publish_period").value,
            callback=self.publish_news)
        self.get_logger().info("Robot News Station has been started")

    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name) + " from the robot news station."
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()