#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node): 
    def __init__(self):
        super().__init__("quiz_node")
        self.get_logger().info("UAV is Awesome")

def main(args=None):
    rclpy.init(args=args) 
    node = MyNode()  
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()