#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def callback(message):
    twist.linear.x = message.axes[5]*0.5
    twist.angular.z = message.axes[4]*1.0

if __name__ == '__main__':
    rospy.init_node('joy_roomba')
    sub = rospy.Subscriber('joy', Joy, callback, queue_size=1)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()
