#! /usr/bin/env python3
import rclpy
from rclpy.node import Node 

class CounterNode(Node):
    def __init__(self, name, period=0.5, verbose=True):
        super().__init__(node_name=name) 
        self.counter = 0
        self.create_timer(timer_period_sec=period, callback=self.timer_callback)

        if verbose:
            self.get_logger().info("Node succesfully created!") # Print something 
        

    def timer_callback(self): # Function which is repeated with the desired frequency
        self.counter += 1
        self.get_logger().info("Counter: %d"%(self.counter))


def main(args=None):
    rclpy.init(args=args) # Starts the ROS2 communication.
    node = CounterNode(name="py_counter", period=1) # Create the ROS2 NODE

    rclpy.spin(node) # Will pause the program and allow the node to continue to be alive (
                     # useful when there are callbacks)
    rclpy.shutdown() # Last line of the program

if __name__=="__main__":
    main()