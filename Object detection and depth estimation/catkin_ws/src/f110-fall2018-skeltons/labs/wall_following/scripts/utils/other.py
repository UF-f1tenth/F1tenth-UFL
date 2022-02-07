"""
Created on Fri Oct 29 18:54:18 2021

@author: Krishna Nuthalapati
"""

import numpy as np

def iou(boxA, boxB):
	# determine the (x, y)-coordinates of the intersection rectangle
	xA = max(boxA[0], boxB[0])
	yA = max(boxA[1], boxB[1])
	xB = min(boxA[2], boxB[2])
	yB = min(boxA[3], boxB[3])
	# compute the area of intersection rectangle
	interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
	# compute the area of both the prediction and ground-truth
	# rectangles
	boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
	boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
	# compute the intersection over union by taking the intersection
	# area and dividing it by the sum of prediction + ground-truth
	# areas - the interesection area
	iou_score = interArea / float(boxAArea + boxBArea - interArea)
	# return the intersection over union value
	return iou_score

def nms(boxes, scores, thresh):
    num_boxes = boxes.shape[0]
    indices   = np.zeros((num_boxes), dtype=int)
    # print("PRINTING : ", num_boxes)
    for i in range(num_boxes):
        if indices[i] == -1:
            continue
        for j in range(i+1, num_boxes):
            if indices[j] == -1:
                continue
            
            base_box = boxes[i]
            curr_box = boxes[j]
            iou_score = iou(base_box, curr_box)
            
            if iou_score >= thresh:
                if scores[i]>scores[j]:
                    indices[i] = 1
                    indices[j] = -1
                    continue
                indices[j] = 1
                indices[i] = -1
    
    idxs = np.where(indices == 1)[0]
    
    return idxs
