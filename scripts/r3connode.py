#!/usr/bin/env python3
import time
import rospy
from f1.msg import currentposes
from geometry_msgs.msg import Twist
import math

pub = rospy.Publisher('r3/vel_controller/cmd_vel',Twist,queue_size=1)

cp = [0,0,0]
wp = [0,0,0]
change = False

v = 0
w = 0

control_signal = Twist()
class PID:

    def __init__(self, saturation ,P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.saturation = saturation
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def set_setpoint(self,set_point):
        self.set_setpoint = set_point

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

            if self.output > self.saturation:
                self.output = self.saturation
            elif self.output < -self.saturation:
                self.output = self.saturation
        return self.output

translation = PID(0.1)
orientation = PID(0.3)

def get_current_positions(posedata):
    global cp
    cp = [posedata.pose3.x,posedata.pose3.y,posedata.pose3.theta]
    

def get_viapoints(wpoints):
    global wp,change
    last = wp
    wp = [wpoints.pose3.x,wpoints.pose3.y,wpoints.pose3.theta]
    if last != wp:
        change = True

def agg():
    global v, w, cp, wp, change
    trans_cp = math.sqrt(pow(cp[0],2)+pow(cp[1],2))
    trans_wp = math.sqrt(pow(wp[0],2)+pow(wp[1],2))
    if change == True:
        translation.clear()
        orientation.clear()
        translation.set_setpoint(trans_wp) # Way point goes here
        orientation.set_setpoint(wp[2]) # Way point for orientation
    v = translation.update(trans_cp)
    w = orientation.update(cp[2])
    control_signal.linear.x = v
    control_signal.angular.z = w
    pub.publish(control_signal)
    return

def main():
    rospy.init_node('r1connode')
    sub1 = rospy.Subscriber('pose_details',currentposes,callback = get_current_positions)
    sub2 = rospy.Subscriber('waypoints',currentposes,callback = get_viapoints)
    while not rospy.is_shutdown():
        agg()
    rospy.spin()

if __name__ == '__main__':
    main()