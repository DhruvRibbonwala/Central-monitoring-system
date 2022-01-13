#!/usr/bin/env python3
import time
import rospy
from f1.msg import currentposes
from geometry_msgs.msg import Twist
import math

pub = rospy.Publisher('r1/vel_controller/cmd_vel',Twist,queue_size=1)

cp = [0,0,0]
wp = [0,0,0]
last = [0,0,0]
change = False

v = 0
w = 0

control_signal = Twist()
class PID:

    def __init__(self, saturation, caller ,P=1, I=0.1, D=0.1, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.caller = caller

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
        self.SetPoint = set_point

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
            #print(self.caller,self.output)

            if self.output > self.saturation:
                self.output = self.saturation
            elif self.output < -self.saturation:
                self.output = -self.saturation
            
            # if self.caller == 'O':
            #     print(self.caller,self.SetPoint,feedback_value,error,self.output)
        return self.output

translation = PID(0.09,'T', P=0.8,I=0.1,D=0.1)
orientation = PID(0.09,'O',P=0.2,I=0.01,D=0.1)

def get_current_positions(posedata):
    global cp
    cp = [posedata.pose1.x,posedata.pose1.y,posedata.pose1.theta]
    
    

def get_viapoints(wpoints):
    global wp,change,last
    last = wp
    wp = [wpoints.pose1.x,wpoints.pose1.y,wpoints.pose1.theta]
    if last != wp:
        change = True
    else:
        change = False

def initialise():
    translation.set_setpoint(0.5)
    return
    

def agg():
    global v, w, cp, wp, change
    trans_cp = math.sqrt(pow(cp[0],2)+pow(cp[1],2))
    trans_wp = math.sqrt(pow(wp[0],2)+pow(wp[1],2))
    #print(trans_wp,trans_cp)
    if change == True:
        print('shift happened')
        translation.clear()
        orientation.clear()
        v = 0
        w = 0
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
    sub1 = rospy.Subscriber('/pose_details',currentposes,callback = get_current_positions)
    sub2 = rospy.Subscriber('/waypoints',currentposes,callback = get_viapoints)
    while not rospy.is_shutdown():
        #print(wp,last,change)
        agg()
        time.sleep(0.7)
    rospy.spin()

if __name__ == '__main__':
    main()