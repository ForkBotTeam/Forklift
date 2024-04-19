#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Int8

import cv2
from pyzbar.pyzbar import decode
import numpy as np
import os
import uuid
import math

'''import torch'''
import freenect

import jetson_inference
import jetson_utils

'''def YOLO_Centroid(boxes): # Centroids and confidences of objects detected by yolo
    centroid=[]
    for box in boxes:
        xt, yt, xb, yb, conf = box
        width = abs(xb - xt)
        height = abs(yb - yt)
        center_x = xt + width / 2
        center_y = yt + height / 2
        centroid.append([(center_x, center_y),conf]) # This list contains the centroids and confidence of objects detected by yolo  <--- This is what you need
    return centroid'''

'''def JETSON_Centroid(detections): # Centroids, confidences and class names of objects detected by jetson
    centroid=[]
    for detect in detections:
        centroid.append([detect.Center[0], detect.Center[1],detect.Confidence,net.GetClassDesc(detect.ClassID)])
    return np.array(centroid) # This numpy array contains the centroids, confidences and class names of objects detected by jetson <--- This is what you need'''
    
def QR_Data(Data,data_saved) : # Append new data and centroids of QR codes detected
    if Data not in data_saved:
        '''path=os.path.join('images','detect',f'{uuid.uuid1()}.jpg')'''
        data_saved.append(Data)
        '''cv2.imwrite(path,cframe)'''

def QR_Centroid_Update(centers,pts): # Update the centroids of QR codes detected
    centers.append([np.mean(pts[0],axis=0)[0],np.mean(pts[0],axis=0)[1]])
    return centers 

def QR_Draw_Bounds(frame,points_saved): # Draw the bounds of QR codes detected

    for pts,pts2 in points_saved:
        cv2.polylines(frame,[pts],True,(255,0,255),5)
        # cv2.putText(frame,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
    return frame

def destination_decoder(codes):
    codes = decode(frame)    
    if codes != None:
        msg = MoveBaseActionGoal()
        for code in codes:
            Data=code.data.decode('utf-8').split(',')
            x = float(Data[0][2:])
            y = float(Data[1][2:])
            orientation = float(Data[2][2:])*np.pi/180
            # Euler angles (in radians)
            roll = 0
            pitch = 0

            # Calculate components
            cos_roll = float(math.cos(roll / 2))
            sin_roll = float(math.sin(roll / 2))
            cos_pitch = float(math.cos(pitch / 2))
            sin_pitch = float(math.sin(pitch / 2))
            cos_yaw = float(math.cos(orientation / 2))
            sin_yaw = float(math.sin(orientation / 2))

            ww = float(cos_pitch * cos_yaw * cos_roll + sin_pitch * sin_yaw * sin_roll)
            xx = float(cos_pitch * cos_yaw * sin_roll - sin_pitch * sin_yaw * cos_roll)
            yy = float(cos_pitch * sin_yaw * cos_roll + sin_pitch * cos_yaw * sin_roll)
            zz = float(sin_pitch * cos_yaw * cos_roll - cos_pitch * sin_yaw * sin_roll)
            msg.goal.target_pose.header.frame_id="map"
            msg.goal.target_pose.pose.position.x=x
            msg.goal.target_pose.pose.position.y=y
            msg.goal.target_pose.pose.orientation.z=yy
            msg.goal.target_pose.pose.orientation.w=-ww

            pts=np.array([code.polygon],np.int32)
            pts.reshape((-1,1,2))

            QR_Data(Data,data_saved) # Append new data and centroids of QR codes detected
            qr_centers = QR_Centroid_Update(qr_centers, pts) # Update the centroids of QR codes detected

            pts2=code.rect

            points_saved.append((pts,pts2))

            return msg

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
    d = freenect.sync_get_depth(format = freenect.DEPTH_REGISTERED)

    if d is not None:
        depth_array, _ = d
        cv2.medianBlur(depth_array, 5)
        visualize_array = cv2.normalize(depth_array,dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        visualize_array = visualize_array.astype(np.uint8)
    else:
        print("failed...retrying")
        return Exception
    return depth_array, visualize_array

def how_far(depth, x, y, **kwargs): # "depth" are kinect depth frame, x,y are pixel locations
    # Remember x-axis translates to indexing of coloumns while y-axis is indexing of rows!
    dist = str(depth[int(y),int(x)]/1000)

    if "print" in kwargs:
        cv2.putText(kwargs["print"], dist,  (int(x+5),int(y+5)),fontFace= 3, fontScale= 0.75, color= (150,20,20),thickness= 2)    
    return dist

if __name__== "__main__":
    '''model=torch.hub.load('ultralytics/yolov5','yolov5s')'''
    net=jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

    rospy.init_node("beacon")

    qr_destination = rospy.Publisher('/destination',MoveBaseActionGoal, queue_size=10)
    object_flag = rospy.Publisher('mish_faker', Int8, queue_size=10)


    rate = rospy.Rate(1)
    k= 0
    data_saved=[]
    points_saved=[]
    qr_centers=[]

    while k!=27 or rospy.is_shutdown(): # quit program when 'esc' key is pressed or terminal terminates
        tic = rospy.get_time()
        k = cv2.waitKey(1) & 0xFF

        # 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱
        # GET FRAMES FROM CAMERA
        #####################################
        frame = get_video() #get a frame from RGB camera
        cuda_frame = jetson.utils.cudaFromNumpy(frame) #convert frame to cuda for jetson inference
        quantitave_depth, visual_depth = get_depth() #get a frame from depth sensor
        ####################################


        # 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱
        # QR-CODE MANEGMENT
        ####################################
        dest = destination_decoder(frame)
        if dest != None:
            qr_destination.publish(dest)
        frame = QR_Draw_Bounds(frame,points_saved) # Draw the bounds of QR codes detected
        ####################################

        
        # 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱 🂱
        # OBJECT DETECTION AND MANEGMENT
        ####################################
        '''results = model(frame)'''
        JETSON_detections = net.Detect(cuda_frame)
        JETSON_detect_frame = jetson_utils.cudaToNumpy(cuda_frame)

        '''cake = results.xyxy[0][:, :-1].cpu().numpy()
        detected_names = np.array(results.pandas().xyxy[0]['name'])'''
        JETSON_cake = JETSON_Centroid(JETSON_detections) # Centroids, confidences and class names of objects detected by jetson
        JETSON_detected_names = JETSON_cake[:,3]
        
        target_object = "person"
        '''indices = np.where(detected_names == target_object)'''
        JETSON_indices = np.where(JETSON_detected_names == target_object)

        # print(f"{detected_names[indices]} \n {cake[indices,:]} \n +++++++++")
        
        '''if indices != None:
            object_flag.publish(ord("S"))
        yolo_centers = YOLO_Centroid(cake)'''
        if JETSON_indices != None:
            object_flag.publish(ord("S"))
        ####################################


        # Print depth of objects and qr_codes on image
        '''for qr_center in qr_centers:
            far = how_far(quantitave_depth, qr_center[0], qr_center[1])
            cv2.putText(frame, far,(int(qr_center[0]), int(qr_center[1])),fontFace= 2, fontScale= 0.75, color= (20,20,150),thickness= 1)'''
        
        for qr_center in qr_centers:
            far = how_far(quantitave_depth, qr_center[0], qr_center[1])
            cv2.putText(JETSON_detect_frame, far,(int(qr_center[0]), int(qr_center[1])),fontFace= 2, fontScale= 0.75, color= (20,20,150),thickness= 1)

        '''for yolo_center in yolo_centers:
            far = how_far(quantitave_depth, yolo_center[0][0], yolo_center[0][1])
            # print(f"{yolo_center[0][0]}  {yolo_center[0][1]}")
            cv2.putText(frame, far,(int(yolo_center[0][0]), int(yolo_center[0][1])),fontFace= 2, fontScale= 0.75, color= (20,20,150),thickness= 1)'''
            
        for JETSON_center in JETSON_cake[:,:2].astype(int):
            far = how_far(quantitave_depth, JETSON_center[0], JETSON_center[1])
            cv2.putText(JETSON_detect_frame, far,(int(JETSON_center[0]), int(JETSON_center[1])),fontFace= 2, fontScale= 0.75, color= (20,20,150),thickness= 1)

            
        '''cv2.imshow('Window',np.squeeze(results.render()))'''
        cv2.imshow('JETSON Window',JETSON_detect_frame)
        tictoc = rospy.get_time() - tic
        print(tictoc)
        rate.sleep()
    
    freenect.sync_stop()
    cv2.destroyAllWindows()

    