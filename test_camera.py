#!/usr/bin/env python

import numpy as np
import rospy
import cv2
import glob
import cv2.aruco as aruco
import math
import sys
sys.path.append('/home/yihernong/kinova_ws/src/kinova-ros/PosePerturbationBenchmark/pose_perturbation_benchmark/scripts')
#from kinova_514_testing import robot
from std_msgs.msg import String
import time


#### Camera caliberation start#####
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('/home/yihernong/Pictures/Cam_cal_514/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray1 = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray1, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray1,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)


# Get all the relevant matrices
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray1.shape[::-1],None,None)
# f = open("cameramatrixcalibration.txt", "a")
# f.write(mtx)
# f.write(dist)
# f.close()

print('Camera Matrix:')
print(mtx)

print('Distortion Matrix:')
print(dist)




### Camera caliberation End #####
rospy.init_node('webcam')
rate = rospy.Rate(10)

#set up publisher
pub = rospy.Publisher('webcam',String,queue_size=1)
# Aruco Marker Info
#marker_id = 11 #ID of generated marker
marker_size = 2 #cm

# Define Aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
parameters = aruco.DetectorParameters_create()

# Capture video from webcam
#cap = cv2.VideoCapture(1)
##if ret:
 #   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#else:
 #   print ("ERROR!!!!!")
start = time.time()

m1c1 = []
m1c2 = []
marker_1 = []
marker_2 = []

while not rospy.is_shutdown():
    # Read camera
    #cap = cv2.VideoCapture(1)
    #cap.release()
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    #print("I AM FINE")
    # Convert in gray scale
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        print("Error")
    #Find markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                             cameraMatrix=mtx, distCoeff=dist)

    #if ids != None and ids[0] == marker_id:
    
   

    if time.time() - start < 10.0:
        if np.all(ids != None):

            rvec, tvec , __ = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)

            # Array containing list of rotation and position of marker in camera frame
            #rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            for i in range(ids.size):
                if i == 0:
                    marker_1 = corners[i][0,0]
                    m1c1 = marker_1
                    m1c2 = corners[i][0,1]
                if i == 1:
                    marker_2 = corners[i][0,0]
                # Draw reference frame of the marker
                aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 2)

            # Find the distance/pixel ratio
            #dist_corners = math.sqrt((m1c1[0] - m1c2[0])**2 + (m1c1[1] - m1c2[1])**2)
            #print(dist_corners)
            #r = dist_corners/float(marker_size)
            aruco.drawDetectedMarkers(frame, corners)

            



        #if ids.size > 0: # Marker ID 
            # del_x = marker_1[0] - marker_2[0]
            # del_y = marker_1[1] - marker_2[1]
            
            #angle = math.atan(del_y/del_x)
            # dist_markers = math.sqrt((marker_1[0] - marker_2[0]) ** 2 + (marker_1[1] - marker_2[1]) ** 2)
            # cv2.line(frame, (marker_1[0], marker_1[1]), (marker_2[0], marker_2[1]), (0, 255, 0), thickness=3,
            #          lineType=8)
            # cv2.putText(frame, 'marker distance: ' + str(dist_markers/r) + 'cm', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            # cv2.putText(frame, 'delta X: ' + str(del_x/r) + 'cm', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            # cv2.putText(frame, 'delta Y: ' + str(del_y / r) + 'cm', (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1,
            #             (255, 0, 0), 2, cv2.LINE_AA)
            # Mapping image coordinates to robot 
            

            # Maker_1 is the image point you want your robot go to
            
            
    
    try:
        #Marker 1 data
        robot_x = ((-0.395/480)*marker_1[1]) - 0.215
        robot_y = ((-0.61/640)*marker_1[0]) + 0.335
        pub.publish(str([robot_x , robot_y, 1]))
        #Marker 2 data
        robot_x = ((-0.395/480)*marker_2[1]) - 0.215
        robot_y = ((-0.61/640)*marker_2[0]) + 0.335
        pub.publish(str([robot_x , robot_y, 2]))

    except: 
        print ("Could not detect markers!!!")
        print("Marker1", marker_1, "Marker2", marker_2)






    #Display 
    cv2.imshow('frame', frame)
    
    

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break


# #Control arm stuff
# 

# Robot.move_to_Goal([0,0.7,0.2,90,165,180])
# Robot.move_to_Goal([-.3875,-.1,0.2,90,165,180])
# Robot.move_to_Goal([-.3875,-.1,0.05,90,165,180])

cap.release()
cv2.destroyAllWindows()