#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def callback_turtle1(pose):
    rospy.loginfo("Turtle1 -> x=%.2f, y=%.2f, θ=%.2f", pose.x, pose.y, pose.theta)

def callback_turtle2(pose):
    rospy.loginfo("Turtle2 -> x=%.2f, y=%.2f, θ=%.2f", pose.x, pose.y, pose.theta)

def leer_pose():
    rospy.init_node('leer_pose_dos_tortugas', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback_turtle1)
    rospy.Subscriber('/t2/pose', Pose, callback_turtle2)
    rospy.spin()

if __name__ == '__main__':
    leer_pose()
