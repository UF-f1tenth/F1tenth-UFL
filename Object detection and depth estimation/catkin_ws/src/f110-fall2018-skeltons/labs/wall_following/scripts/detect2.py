#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized

import numpy as np
import pyzed.sl as sl
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

@torch.no_grad()
def detect():
    source = '0'
    weights = '/home/f1tenth2/f110_ws/src/f110-fall2018-skeletons/labs/wall_following/scripts/yolov3-tiny-best.pt'
    imgsz = 416
    webcam = True
    conf_thres = 0.25
    iou_thres  = 0.45
    agnostic_nms = False
    classes = None 

    # Initialize
    device = select_device('')
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    if half:
        model.half()  # to FP16

    # Set Dataloader
    view_img = check_imshow()
    cudnn.benchmark = True  # set True to speed up constant image size inference
    #dataset = LoadStreams(source, img_size=imgsz, stride=stride)

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

    counter = 1
    #for path, img, im0s, vid_cap in dataset:
    key = ''
    while key != 113:  # for 'q' key
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat)
            im0s = mat.get_data()
        img = cv2.cvtColor(im0s, cv2.COLOR_RGBA2BGR)
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
        text = f'Inference speed : {t2 - t1:.3f}s ({1/(t2-t1):.3} fps)'

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                s, im0 = f'{i}: ', im0s[i].copy()

            s += '%gx%g ' % img.shape[2:]  # print string
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    cl= (0,0,255) if names[c] == 'red' else (0, 255, 0)
                    label = f'{names[c]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=cl, line_thickness=3)
                    

            # Print time (inference + NMS)
            im0 = cv2.putText(im0, text, (10, im0.shape[0]-28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
            im0 = cv2.putText(im0, "Model : yolov3-tiny", (10, im0.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
            print(text)
            cv2.imshow("ZED DETECTION", im0)
            #cv2.imwrite("/media/f1tenth2/32gb-sd/data_collection/{}.jpg".format(counter), im0)
            counter+=1
            cv2.waitKey(1)  # 1 millisecond

if __name__ == '__main__':
    check_requirements(exclude=('tensorboard', 'pycocotools', 'thop'))
    detect()
    
    
    
    
    
    
