#!/usr/bin/env python 3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNone(Node):

    def __init__(self): #constructor of the node
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) 
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        # ^buffer of 10 messages
        self.get_logger().info("Draw circle nonde has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0 
        #this is the data of the message, found through $ros2 interface show geometry_msgs/msg/Twist
        self.cmd_vel_pub_.publish(msg) #publish method to send message

def main(args=None):
    rclpy.init(arg=args)
    node = DrawCircleNone() #runs the node
    rclpy.spin(node) #spins the node 
    rclpy.showdown()

