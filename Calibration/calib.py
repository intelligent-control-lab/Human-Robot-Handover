# This is the program for calibrating the camera position relative to the robot.
# Copyright (C) 2022  

# Authors:
# Ruixuan Liu: ruixuanl@andrew.cmu.edu
# Rui Chen: ruic3@andrew.cmu.edu
# Changliu Liu : cliu6@andrew.cmu.edu

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

# Camera Intrinsic parameters.
cam_mtx = np.matrix([[1.0327407219495085e+03, 0., 9.5685930301206076e+02], 
                     [0., 1.0323427647512485e+03, 5.3914133979587950e+02],
                     [0., 0., 1. ]])
dist_coeff = np.array([2.0429028189430772e-02, -7.8399448855923665e-03, -3.0949667668667908e-03, 2.1848647826377197e-03, -5.2344198896187882e-02 ])

# Side length of the ArUco marker in meters. Manually measured.
aruco_marker_side_length = 0.172

# Tag position in world frame. Manually measured.
robot_to_tag = np.matrix([[1, 0, 0, 0.53],
                          [0, 1, 0, 0.33],
                          [0, 0, 1, 0.87],
                          [0, 0, 0, 1]])

aruco_dictionary_name = "DICT_ARUCO_ORIGINAL"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])

img = cv2.imread("calib.jpg")

for key in ARUCO_DICT:
	this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[key])
	(corners, marker_ids, rejected) = cv2.aruco.detectMarkers(img,this_aruco_dictionary,cameraMatrix=cam_mtx, distCoeff=dist_coeff)
	if(marker_ids != None):
		break

rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_marker_side_length, cam_mtx, dist_coeff)

rot = R.from_rotvec(np.reshape(rvecs, (3,)))
translation = np.reshape(tvecs, (3,))
rot = rot.as_matrix()
cv2.aruco.drawAxis(img, cam_mtx, dist_coeff, rvecs, tvecs, 0.05)
camera2tag = np.identity(4)
camera2tag[:3, :3] = rot
camera2tag[:3, 3] = translation

tag2camera = np.linalg.inv(camera2tag)
transformation = np.matmul(robot_to_tag, tag2camera)
np.savetxt("camera_transformation.txt", transformation)

cv2.imshow('f', img)
cv2.waitKey(0)