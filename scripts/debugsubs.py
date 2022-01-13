#!/usr/bin/env python3
import rospy
from f1.msg import currentposes

def cb1(data):
    rospy.loginfo(data)

# def cb2(data):
#     rospy.loginfo(data)

def listener():
    rospy.init_node('degubsubs')
    rospy.Subscriber("pose_details",currentposes,callback=cb1)
    #rospy.Subscriber("waypoints",currentposes,callback=cb2)
    rospy.spin()

if __name__ == '__main__':
    listener()