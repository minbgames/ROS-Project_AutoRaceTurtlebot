#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

target_linear_vel = 0
target_angular_vel = 0

def callback(msg):
    target_linear_vel = msg.data[0]
    target_angular_vel = msg.data[1]

    twist = Twist()
    twist.linear.x = target_linear_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_angular_vel
    pub.publish(twist)


def Main():
    rospy.init_node('turtlebot3_teleop')
    rospy.Subscriber('TELEOP_msg', Float32MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    Main()
