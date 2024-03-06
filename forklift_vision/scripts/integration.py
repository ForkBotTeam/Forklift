#!/usr/bin/env python3
from rospy import is_shutdown
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import os
import uuid
import torch
import freenect

def YOLO_Centroid(boxes): # Centroids and confidence of objects detected by yolo
    centroid=[]
    for box in boxes:
        xt, yt, xb, yb, conf = box
        width = abs(xb - xt)
        height = abs(yb - yt)
        center_x = xt + width / 2
        center_y = yt + height / 2
        centroid.append([(center_x, center_y),conf]) # This list contains the centroids and confidence of objects detected by yolo  <--- This is what you need
    return centroid   

def QR_Data(Data,data_saved) : # Append new data and centroids of QR codes detected
    if Data not in data_saved:
        path=os.path.join('images','detect',f'{uuid.uuid1()}.jpg')
        data_saved.append(Data)
        cv2.imwrite(path,cframe)

def QR_Centroid_Update(centers,pts): # Update the centroids of QR codes detected
    centers.append([np.mean(pts[0],axis=0)[0],np.mean(pts[0],axis=0)[1]])
    return centers 

def QR_Draw_Bounds(frame,points_saved): # Draw the bounds of QR codes detected

    for pts,pts2 in points_saved:
        cv2.polylines(frame,[pts],True,(255,0,255),5)
        # cv2.putText(frame,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
    return frame

# Kinect camera normal and depth frame retrievel 
def get_video(): #function to get RGB image from kinect
    d = freenect.sync_get_video()
    if d is not None:
        array, _ = d
        array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    else:
        return Exception
    return array
def get_depth(): #function to get depth image from kinect
    d = freenect.sync_get_depth()
    if d is not None:
        array, _ = d
        array = array.astype(np.uint8)
    else:
        return Exception
    return array
def how_far(depth, x, y): # "depth" are kinect depth frame, x,y are pixel locations
    return np.round(depth[int(y), int(x)] + 65, 1) # cm


if __name__== "__main__":
    model=torch.hub.load('ultralytics/yolov5','yolov5s')

    # cv2.moveWindow('Window',0,0)

    k= 0
    while k!=27 or is_shutdown(): # quit program when 'esc' key is pressed or terminal terminates
        data_saved=[]
        points_saved=[]
        qr_centers=[]
        k = cv2.waitKey(1) & 0xFF

        frame = get_video() #get a frame from RGB camera
        depth = get_depth() #get a frame from depth sensor

        codes = decode(frame)
        if len(codes)!=0:
            for code in codes:
                cframe=frame.copy()
                Data=code.data.decode('utf-8')

                pts=np.array([code.polygon],np.int32)
                pts.reshape((-1,1,2))

                QR_Data(Data,data_saved) # Append new data and centroids of QR codes detected
                qr_centers = QR_Centroid_Update(qr_centers, pts) # Update the centroids of QR codes detected
                cv2.polylines(cframe,[pts],True,(255,0,255),5)
                pts2=code.rect

                points_saved.append((pts,pts2))

                cv2.putText(cframe,Data,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
        
        # print(f'QR Centroids : {qr_centers}') # This list contains the centroids of QR codes detected <-- This is what you need "QRs detected from before are not deleted"
        frame = QR_Draw_Bounds(frame,points_saved) # Draw the bounds of QR codes detected

        results = model(frame)
        cake = results.xyxy[0][:, :-1].cpu().numpy()
        yolo_centers = YOLO_Centroid(cake)

        # print(f'YOLO centroids : {yolo_obj}') # This list contains the centroids and confidence of objects detected by yolo  <--- This is what you need "Objects detected from before are deleted if not detected"
        
        # Print depth of objects and qr_codes on image
        for qr_center in qr_centers:
            far = how_far(depth, qr_center[0], qr_center[1])
            cv2.putText(frame, str(far),(int(qr_center[0]), int(qr_center[1])),fontFace= 2, fontScale= 0.75, color= (20,20,150),thickness= 1)

        for yolo_center in yolo_centers:
            far = how_far(depth, yolo_center[0][0], yolo_center[0][1])
            # print(f"{yolo_center[0][0]}  {yolo_center[0][1]}")
            cv2.putText(frame, str(far),(int(yolo_center[0][0]), int(yolo_center[0][1])),fontFace= 2, fontScale= 0.75, color= (20,20,150),thickness= 1)

            
        cv2.imshow('Window',np.squeeze(results.render()))
    
    freenect.sync_stop()
    cv2.destroyAllWindows()










