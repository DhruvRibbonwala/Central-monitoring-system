#!/usr/bin/env python3
import rospy
import time
from nav_msgs.msg import Odometry
from f1.msg import currentposes
from tf.transformations import euler_from_quaternion

fhand = open('logs.txt','w')
fhand2 = open('wps.txt','w')
start = time.time()

def odom_callback(msg):
    global start, fhand
    angles = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    current_time = time.time()
    td = current_time - start
    final_string = "{} {} {} {}\n".format(td,msg.pose.pose.position.x,msg.pose.pose.position.y,angles[2])
    #print(final_string)
    fhand.write(final_string)

def get_viapoints(wpoints):
    global fhand2
    wp = [wpoints.pose1.x,wpoints.pose1.y,wpoints.pose1.theta]
    current_time = time.time()
    td = current_time - start
    wp_string = "{} {} {} {}\n".format(td,wp[0],wp[1],wp[2])
    fhand2.write(wp_string)


def main():
    rospy.init_node('lognode')
    print('Node declared')
    rospy.Subscriber('r1/r1/vel_controller/odom',Odometry,odom_callback)
    rospy.Subscriber('waypoints',currentposes,get_viapoints)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print('exception')
        pass 