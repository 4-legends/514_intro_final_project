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
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)


# Get all the relevant matrices
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

# dict_cam = {ret, mtx, dist, rvecs, tvecs}

np.save('ret.npy', ret) #np.array([ret, mtx, dist, rvecs, tvecs]))
np.save('mtx.npy', mtx)
np.save('dist.npy', dist)
np.save('rvecs.npy', rvecs)
np.save('tvecs.npy', tvecs)

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
cap = cv2.VideoCapture(0)
pic_no = 0
while not rospy.is_shutdown():
    # Read camera
    ret, frame = cap.read()
    #print(ret,frame)
    print(len(frame))
    # img = cv2.imread(frame)
    # print(img)

            
    #Display 
    cv2.imwrite('images/frame_{}.png'.format(pic_no), frame)
    rospy.sleep(1)
    pic_no +=1

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

 
cap.release()
cv2.destroyAllWindows()