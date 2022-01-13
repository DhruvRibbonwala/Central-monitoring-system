#!/usr/bin/env python3
import math
import cv2 as cv
import cv2.aruco as aruco
import glob
import os.path
import rospy
from f1.msg import currentposes
import time

last_ids = [0,0,0,0]

def create_msg(final_poses):
    #print(type(final_poses))
    posemsg = currentposes()
    posemsg.id1 = 1
    posemsg.pose1.x = final_poses[1][0]
    posemsg.pose1.y = final_poses[1][1]
    posemsg.pose1.theta = final_poses[1][2]
    posemsg.id2 = 2
    posemsg.pose2.x = final_poses[2][0]
    posemsg.pose2.y = final_poses[2][1]
    posemsg.pose2.theta = final_poses[2][2]
    posemsg.id3 = 3
    posemsg.pose3.x = final_poses[3][0]
    posemsg.pose3.y = final_poses[3][1]
    posemsg.pose3.theta = final_poses[3][2]
    posemsg.id4 = 4
    posemsg.pose4.x = final_poses[4][0]
    posemsg.pose4.y = final_poses[4][1]
    posemsg.pose4.theta = final_poses[4][2]
    return posemsg


class vision_node:
    def __init__(self,total_bots = 4):
        self.corners = []
        self.ids = []
        self.total_bots = total_bots
        self.ids_list = [1,2,3,4]
        self.newposes = {k:[] for k in self.ids_list}
        # Initialise with the first pose
        self.finalposes = {1:[0.0,0.0,0.785],2:[0.0,0.1524,0.785],3:[0.0,0.3048,0.785],4:[0.0,0.4572,0.785]}

    def track_aruco(self,img_path):
        print(img_path)
        arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        arucoParams = aruco.DetectorParameters_create()
        try:
            img = cv.imread(img_path)
            (corners, ids, _) = aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
        except:
            return False
        self.corners = corners
        self.ids = ids
        os.remove(img_path)
        #print(self.corners,self.ids)
        return True

    def map_coordinates(self, ind_corners,img_len = 500, x_shift = 557, y_shift = 79.5, scale_fact = 0.00393356):
        single_pose = []
        # Shifting y axis up for calculations
        ul = (int(ind_corners[0][0][0]), img_len - int(ind_corners[0][0][1]))
        ur = (int(ind_corners[0][1][0]), img_len - int(ind_corners[0][1][1]))
        lr = (int(ind_corners[0][2][0]), img_len - int(ind_corners[0][2][1]))
        ll = (int(ind_corners[0][3][0]), img_len - int(ind_corners[0][3][1]))

        # Calculating angles or orientations
        num = ul[1] - ll[1]
        den = ul[0] - ll[0]

        if (num * den) > 0 and num != 0 and den != 0:
            angle = (1.57 - math.atan(float(num/den))) * -1.0
        elif (num * den) < 0 and num != 0 and den != 0:
            angle = 1.57 + math.atan(float(num/den))
        elif num == 0 and den > 0:
            angle = 1.57
        elif num == 0 and den < 0:
            angle = -1.57
        elif den == 0:
            angle = 0

        # Remapping to the actual simulation coordinates
        x = (x_shift - (ul[0] + ur[0] + lr[0] + ll[0]) / 4) * scale_fact
        y = (((ul[1] + ur[1] + lr[1] + ll[1]) / 4) - y_shift) * scale_fact

        single_pose = [y, x, angle]
        #poses[list(poses.keys())[i]] = pose
        return single_pose

    def arrange_ids(self):
        global last_ids
        fp = {1:[],2:[],3:[],4:[]}
        corners = [k.tolist() for k in self.corners]
        try:
            ids = [k[0] for k in self.ids]
        except:
            ids = last_ids
        current_poses = dict(zip(ids,corners))
        for key in current_poses:
            self.newposes[key] = vision_node.map_coordinates(self,current_poses[key])
        for i in self.ids_list:
            if len(self.newposes[i]) == 0:
                fp[i] = self.finalposes.get(i)
            else:
                fp[i] = self.newposes.get(i)

        last_ids = ids
        self.finalposes = fp
        return self.finalposes
    


# def main():
#     time.sleep(4.5) # Best 4.5
#     pub = rospy.Publisher('pose_details',currentposes, queue_size=1)
#     rospy.init_node('vision_node')
#     #r = rospy.Rate(5)
#     ob = vision_node()
#     cnt = 0
#     while not rospy.is_shutdown():
#         if cnt < 10:
#             file_path = '/home/dhruv/cv/sim3/arena_camera_1_camera_link_1_camera_1(1)-000{}.jpg'.format(cnt)
#         elif cnt>=10 and cnt < 100:
#             file_path = '/home/dhruv/cv/sim3/arena_camera_1_camera_link_1_camera_1(1)-00{}.jpg'.format(cnt)
#         elif cnt >= 100 and cnt <1000:
#             file_path = '/home/dhruv/cv/sim3/arena_camera_1_camera_link_1_camera_1(1)-0{}.jpg'.format(cnt)
#         elif cnt >= 1000:
#             file_path = '/home/dhruv/cv/sim3/arena_camera_1_camera_link_1_camera_1(1)-{}.jpg'.format(cnt)
#         ob.track_aruco(file_path)
#         final_poses = ob.arrange_ids()
#         posemsg = create_msg(final_poses)
#         pub.publish(posemsg)
#         cnt = cnt + 1
#         if cnt >= 3:
#             time.sleep(0.7) # Best 0.7
#         else:
#             time.sleep(1.2) # Best 1.2


def main():
    last_file = ''
    pub = rospy.Publisher('pose_details',currentposes, queue_size=1)
    rospy.init_node('vision_node')
    folder_path = r'/home/dhruv/cv/sim3'
    file_type = '/*jpg'
    ob = vision_node()
    while not rospy.is_shutdown():
        files = glob.glob(folder_path+file_type)
        try:
            max_file = max(files,key=os.path.getctime)
        except: continue
        if max_file != last_file:
            flag = ob.track_aruco(max_file)
            if flag == False:
                continue
            elif flag == True: pass
            final_poses = ob.arrange_ids()
            posemsg = create_msg(final_poses)
            pub.publish(posemsg)
            last_file = max_file
            time.sleep(0.5)
        else:
            continue
        
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
        # folder_path = r'/home/dhruv/cv/sim3'
        # for f in os.listdir(folder_path):
        #     os.remove(os.path.join(folder_path, f))
