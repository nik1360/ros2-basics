#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from functools import partial # Allows to add more argument to a callback

from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6,7)
    
    def call_add_two_ints_server(self, a, b):
        # Create the client
        client = self.create_client(
            srv_type=AddTwoInts, srv_name="add_two_ints")
        
        # Wait for the service Server to be active
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server Add Two Ints...")

        # Create the request object 
        request = AddTwoInts.Request()
        request.a = a
        request.b = b   

        # Call the server asynchronously
        future = client.call_async(request=request) 
        # Send the request in a non-blocking way and create the future object (i.e., the response)
        future.add_done_callback(callback=partial(self.callback_call_add_two_ints_server, a=a, b=b)) # Add a callback when the futue object is populated (i.e., a response is received)
    
    def callback_call_add_two_ints_server(self, future, a, b):
        # Process the request and does something with it
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + 
                str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()