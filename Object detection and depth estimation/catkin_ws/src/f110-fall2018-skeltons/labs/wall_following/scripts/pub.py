#!/usr/bin/env python3

import cv2 # OpenCV library
import sys, time
import numpy as np
import rospy
from sensor_msgs.msg import String
import pyzed.sl as sl
import os
from ackermann_msgs.msg import AckermannDrive
import math

## Camera Parameters
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

pub = rospy.Publisher('trigger1', String, queue_size=1)
msg = CompressedImage()
  
runtime = sl.RuntimeParameters()
image_mat = sl.Mat()
depth_mat = sl.Mat()
  
def publish_message():
  rospy.init_node('trigger_1', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(4) # 10hz

  # While ROS is still running.
  key = ''
  t1 = 0
  
  while not rospy.is_shutdown():
      err = cam.grab(runtime)
      
      if err == sl.ERROR_CODE.SUCCESS:
        cam.retrieve_image(image_mat)
        image = image_mat.get_data()
        cam.retrieve_measure(depth_mat, sl.MEASURE.XYZRGBA)

        # Publishing image
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        pub.publish(msg)
 
      t = time.time() - t1
      t1 = time.time()
      print(f"Publishing speed : {t:.3} s ({(1/t):.3} fps)")  
      # Sleep just enough to maintain the desired rate
      rate.sleep()


def pub_depth(msg):
  x, y = msg.speed, msg.acceleration
  dist_list = []
  for i in range(-4, 2, 6):
    err, depth_value = depth_mat.get_value(i+int(x*1280), i+int(y*720))
    distance = math.sqrt(depth_value[0] * depth_value[0] +
                       depth_value[1] * depth_value[1] +
                       depth_value[2] * depth_value[2])
    if not np.isnan(distance) and not np.isinf(distance):
      dist_list.append(distance) 
  
  try:
    distance = sum(dist_list)/len(dist_list)                     
  except ZeroDivisionError:
    pass
    
  msg.jerk = float(distance)
  depth_pub.publish(msg)

       
if __name__ == '__main__':
  time.sleep(15)
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
