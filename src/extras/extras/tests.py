#!/usr/bin/env python3

import cv2
import numpy as np


file_path = '/home/sedrica/ros2ppf_26_10/src/extras/track/my_map_new.png'

def extract_yaw_roi(matrix,vect,yaw,input_scale,target_scale):
    '''
    matrix: np array
    vect: 1x2 np array
    yaw: 0 to 360deg
    input_scale: 1x1 np array
    target_scale: 1x1 np array
    '''
    # matrix = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    M = cv2.getRotationMatrix2D((vect[0,0], vect[0,1]), yaw, 1.0)
    matrix[int(vect[0,0]), int(vect[0,1])]=0
    img = cv2.warpAffine(matrix, M, matrix.shape)
    pts = np.array([[-(input_scale/2-1), 0],
                    [-(input_scale/2-1), input_scale-1],
                    [input_scale/2-1, input_scale-1],
                    [input_scale/2-1, 0]],dtype=np.float32)
    pts_shifted = pts + vect
    target_pts=np.array([[0,0],[0,target_scale-1],[target_scale-1,target_scale-1],[target_scale-1,0]],dtype=np.float32)
    M=cv2.getAffineTransform(pts_shifted[[1,0,3]],target_pts[[3,0,1]])
    roi = cv2.warpAffine(img, M, (target_scale, target_scale))
    M = cv2.getRotationMatrix2D((target_scale//2,target_scale//2), 90, 1.0)
    roi_turned = cv2.warpAffine(roi, M, roi.shape)
    return roi_turned

yaw=0
vect = np.array([[249,249]],dtype=np.float32)
roi=extract_yaw_roi(file_path,vect,yaw,500,500)
cv2.imshow('image', roi)
cv2.waitKey(0)