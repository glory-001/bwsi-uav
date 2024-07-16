#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose 

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) #qos is like a buffer of messages

    def pose_callback(self, msg:Pose): #callback is called when a message is received
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")") #print the message we receive

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()