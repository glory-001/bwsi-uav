#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen #service is srv, topic is msg
#remember to add dependancies of turtlesim to the xml
from functools import partial


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x_ = 0
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle controller has been started.")
    
    def pose_callback(self, pose: Pose): #called 60 times per second
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0 
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

        #USING SERVICE TO CHANGE COLOR
        #we DO NOT want to call this 60x per second (second if statement, previous_x_)
        if pose.x > 5.5 and self.previous_x_ <= 5.5: #right side of the screen, red pen
            self.previous_x_ = pose.x
            self.get_logger().info("set color to red :D")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5: #left side of the screen, green pen 
            self.previous_x_ = pose.x
            self.get_logger().info("set color to green :D")
            self.call_set_pen_service(0, 255, 0, 3, 0)
    
    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0): #so we don't get an error
            self.get_logger().warn("Waiting for service...") #warn outputs in yellow
            #create the client, make sure service is up

        request = SetPen.Request() #makes the request
        request.r = r
        request.g = g
        request.b = b 
        request.width = width
        request.off = off

        future = client.call_async(request) #something that will be done in the future
        # ^^returns immedietly (instead of waiting until service has replied, to avoid conflicts with spinning)
        #call the service (send request to server)
        future.add_done_callback(partial(self.callback_set_pen))
        # ^^ calls the callback once the serivce has replied

    def callback_set_pen(self, future): #when the service replies with the response 
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,)) #prints the error

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()