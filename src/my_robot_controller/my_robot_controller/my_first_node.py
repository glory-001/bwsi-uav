#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node): #inherits from Node from rclpy.node

    def __init__(self):
        super().__init__("first_node") #node name, diff from file name
        #self.get_logger().info("Hello from ROS2")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args) #initilize ros communications
    node = MyNode()  
    rclpy.spin(node) #enables all callbacks too
    rclpy.shutdown() #destroys node and everything in it

if __name__ == '__main__': #directly execute file from terminal
    main()