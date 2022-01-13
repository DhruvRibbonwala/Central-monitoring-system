#!/usr/bin/env python3
import time
import rospy
from geometry_msgs.msg import Twist

def main():

    rospy.init_node('debugcontrol')
    pub = rospy.Publisher('r1/vel_controller/cmd_vel',Twist,queue_size=1)
    cs = Twist()
    cs.linear.x = 0.03
    cs.angular.z = 0.0
    while not rospy.is_shutdown():
        pub.publish(cs)
        time.sleep(0.3)

if __name__ == '__main__':
    main()


