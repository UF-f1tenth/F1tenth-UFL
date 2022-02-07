#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
from std_msgs.msg import String
import cv2 # OpenCV library
import numpy as np
import os
os.chdir("/home/f1tenth2/f110_ws/src/f110-fall2018-skeletons/labs/wall_following/scripts")
import module_name as m

import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn

from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized
from ackermann_msgs.msg import AckermannDrive


weights = '/home/f1tenth2/f110_ws/src/f110-fall2018-skeletons/labs/wall_following/scripts/yolov3-tiny-best.pt'
imgsz = 416
webcam = True
conf_thres = 0.25
iou_thres  = 0.45
agnostic_nms = False
classes = None

# Initialize
a = m.some_class_py()
a.initialize_mem_py()

device = select_device('')
half = device.type != 'cpu'  # half precision only supported on CUDA

# Load model
model = attempt_load(weights, map_location=device)  # load FP32 model
stride = int(model.stride.max())  # model stride
imgsz = check_img_size(imgsz, s=stride)  # check img_size
names = model.module.names if hasattr(model, 'module') else model.names  # get class names
if half:
  model.half()  # to FP16

cudnn.benchmark = True  # set True to speed up constant image size inference

# Run inference
if device.type != 'cpu':
  model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

counter=0
t = 0

@torch.no_grad()
def callback(data):
  global counter
  global t

  # Output debugging information to the terminal
  #rospy.loginfo("From sub : " + data.data)
  im0s  = a.get_image_py().astype("uint8", copy=False)
  im0 = im0s.copy()
  depth = a.get_pcl_py().astype("float", copy=False)
  
  img = letterbox(im0s, imgsz, stride=32)[0]
  # Convert
  img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
  img = np.ascontiguousarray(img)
  
  img = torch.from_numpy(img).to(device)
  img = img.half() if half else img.float()  # uint8 to fp16/32
  img /= 255.0  # 0 - 255 to 0.0 - 1.0
  if img.ndimension() == 3:
    img = img.unsqueeze(0)
  
  # Inference
  t1 = time_synchronized()
  pred = model(img, augment=False)[0]

  # Apply NMS
  pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms,
                                   max_det=1000)
  t2 = time_synchronized()
  text1 = f'Inference speed : {t2 - t1:.3f}s ({1/(t2-t1):.3} fps)'
  print(text1)
  
  color = (255,255,255)
  
  for i, det in enumerate(pred):
   
    if len(det):
      det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

      for *xyxy, conf, cls in reversed(det):
        c = int(cls)  # integer class
        cl= (0,0,255) if names[c] == 'red' else (0, 255, 0)
        label = f'{names[c]} {conf:.2f}'
        mid_x = int(((xyxy[0] + xyxy[2])/2).cpu().data.numpy())
        mid_y = int(((xyxy[1] + xyxy[3])/2).cpu().data.numpy())
        x = depth[mid_y][mid_x][0]
        y = depth[mid_y][mid_x][1]
        z = depth[mid_y][mid_x][2]
        dist = np.sqrt(x**2 + y**2 + z**2)
        plot_one_box(xyxy, im0, label="{:.3} m".format(dist), color=cl, line_thickness=3)

  curr_t = time.time()
  text2 = f"Total speed : {curr_t-t:.3} ({1/(curr_t-t):.3} fps)" 
  print(text2, '\n\n')

  im0 = cv2.putText(im0, text1, (10, im0.shape[0]-46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
  im0 = cv2.putText(im0, text2, (10, im0.shape[0]-28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
  im0 = cv2.putText(im0, "Model : yolov3-tiny", (10, im0.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)   
  
  #cv2.imshow("ZED DETECTION", im0)
  counter+=1
  cv2.imwrite("/media/f1tenth2/32gb-sd/data_collection/{}.jpg".format(counter), im0)
  cv2.waitKey(1)
  t = time.time()

  
def receive_message():
  rospy.init_node('object_detection', anonymous=True)
  rospy.Subscriber('chatter_1', String, callback)
  rospy.spin()
  cv2.destroyAllWindows()
  
  
receive_message()
  
  
  
