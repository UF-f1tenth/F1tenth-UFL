#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 28 15:15:54 2021

@author: Krishna Nuthalapati
"""

import pyzed.sl as sl
import cv2
import os

save_path = "/media/f1/32gb-sd/data_collection/"

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER
init_params.sdk_verbose = True

cam = sl.Camera()
status = cam.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

runtime = sl.RuntimeParameters()
mat = sl.Mat()

key = ''
print(" RECORDING ")
counter = 0

while key != 113:  # for 'q' key
	err = cam.grab(runtime)
	if err == sl.ERROR_CODE.SUCCESS:
		cam.retrieve_image(mat)
		image = mat.get_data()
		#cv2.imshow("ZED", image)
		counter+=1
		cv2.imwrite(save_path + str(counter) + ".jpg", image)
		key = cv2.waitKey(1)
	else:
		key = cv2.waitKey(1)
cv2.destroyAllWindows()




