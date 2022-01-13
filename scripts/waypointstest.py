#!/usr/bin/env python3
import rospy
from f1.msg import currentposes
import time
import math

#robot_1_path = [[0,0,0],[0,0,-0.3],[0,0,-0.785],[0,0,-1.571],[0,0,0],[0,0,0.3],[0,0,0.785]]
robot_1_path = [[0.6,0,0],[1.2192,0,0],[1.2192,0,-1.51844],[1.2192,-0.9144,-1.51844],[1.2192,-1.0,0]]
# robot_2_path = [[0,0.1524,0],[1.3716,0.1524,0],[1.3716,0.1524,-1.51844],[1.3716,-1.066,-1.51844],[1.3716,-1.066,0]]
# robot_3_path = [[0,0.3048,0],[1.3716,0.3048,0],[1.3716,0.3048,1.51844],[1.3716,1.524,1.51844],[1.3716,1.524,0]]
# robot_4_path = [[0,0.4572,0],[1.2192,0.4572,0],[1.4572,0.6096,1.51844],[1.2192,1.524,1.51844],[1.2192,1.524,0]]

ids_list = [1,2,3,4]
pose_dict = {k:[0,0,0] for k in ids_list}
finalposes = {1:[0.0,0.0,0.0],2:[0.0,0.0,0.0],3:[0.0,0.0,0.0],4:[0.0,0.0,0.0]}

trans_buffer = 0.05
angle_buffer = math.radians(3)

wpoints = currentposes()
waypub = rospy.Publisher('waypoints',currentposes,queue_size=100)

def sub_points(posedata):
    #global pose_dict
    # COnverting messages to the dictionary
    pose_dict[1][0] = posedata.pose1.x
    pose_dict[1][1] = posedata.pose1.y
    pose_dict[1][2] = posedata.pose1.theta
    pose_dict[2][0] = posedata.pose2.x
    pose_dict[2][1] = posedata.pose2.y
    pose_dict[2][2] = posedata.pose2.theta
    pose_dict[3][0] = posedata.pose3.x
    pose_dict[3][1] = posedata.pose3.y
    pose_dict[3][2] = posedata.pose3.theta
    pose_dict[4][0] = posedata.pose4.x
    pose_dict[4][1] = posedata.pose4.y
    pose_dict[4][2] = posedata.pose4.theta


def publish_points(id,path_pub):
    global finalposes
    if id == 1:
        wpoints.id1 = 1
        wpoints.pose1.x = path_pub[0]
        wpoints.pose1.y = path_pub[1]
        wpoints.pose1.theta = path_pub[2]
        wpoints.id2 = 2
        wpoints.pose2.x = 0
        wpoints.pose2.y = 0
        wpoints.pose2.theta = 0
        wpoints.id3 = 3
        wpoints.pose3.x = 0
        wpoints.pose3.y = 0
        wpoints.pose3.theta = 0
        wpoints.id4 = 4
        wpoints.pose4.x = 0
        wpoints.pose4.y = 0
        wpoints.pose4.theta = 0
        waypub.publish(wpoints)
    return
    

def move_forward(id,path_forward):
    global pose_dict
    current_pose = pose_dict[id]
    cnt = 1
    while (cnt <= len(path_forward)):
        trans_circle = math.sqrt(pow(path_forward[cnt][0]-current_pose[0],2)+pow(path_forward[cnt][1]-current_pose[1],2))
        #in_x_buffer = current_pose[0] >= (path_forward[cnt][0] - trans_buffer) and  current_pose[0] <= (path_forward[cnt][0] + trans_buffer)
        #in_y_buffer = current_pose[1] >= (path_forward[cnt][1] - trans_buffer) and  current_pose[1] <= (path_forward[cnt][1] + trans_buffer)
        in_angle_buffer = current_pose[2] >= (path_forward[cnt][2] - angle_buffer) and  current_pose[2] <= (path_forward[cnt][2] + angle_buffer)
        #reqsum = sum([abs(num) for num in current_pose]) - sum([abs(num) for num in path_forward[cnt]])
        if trans_circle <= trans_buffer and in_angle_buffer == True:
            cnt = cnt + 1
        try:
            publish_pose = path_forward[cnt]
            #pose_dict[id] = path_forward[cnt]
        except IndexError:
            break
        #Publish the value of publish pose list at this exact line or call a function from here
        time.sleep(0.5)
        publish_points(id,publish_pose)
    return

def move_inverse(id,path_forward):
    global pose_dict
    current_pose = pose_dict[id]
    path_inverse = path_forward[::-1]
    cnt = 1
    while (cnt <= len(path_inverse)):
        if sum([abs(num) for num in current_pose]) >= sum([abs(num) for num in path_inverse[cnt]]):
            cnt = cnt + 1
        try:
            publish_pose = path_inverse[cnt]
        except IndexError:
            break
        #Publish the value of publish pose list at this exact line or call a function from here
        time.sleep(0.5)
        publish_points(id,publish_pose)
    return



def main():
    rospy.init_node('waypoints')
    sub = rospy.Subscriber('pose_details',currentposes,sub_points)
    move_forward(1,robot_1_path)
    move_inverse(1,robot_1_path)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass