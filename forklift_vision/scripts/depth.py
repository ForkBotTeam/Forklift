#!/usr/bin/env python3

import freenect
import cv2
import numpy as np

 
def get_video(): #function to get RGB image from kinect
    d = freenect.sync_get_video()
    if d is not None:
        array, _ = d
        array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    else:
        print("failed...retrying")
        return Exception
    return array
 
def get_depth(): #function to get depth image from kinect
    d = freenect.sync_get_depth()
    if d is not None:
        array, _ = d
        array = array.astype(np.uint8)
    else:
        print("failed...retrying")
        return Exception
    return array

def how_far(depth,x,y): # "depth" are kinect depth frame, x,y are pixel locations
    return np.round(depth[x,y] + 65, 1) # cm

if __name__ == "__main__":

    print(np.array(dir(freenect)).transpose())
    print(freenect.get_depth_format(0))
    freenect.freenect_set_tilt_degs(20)
    while 1:
        frame = get_video() #get a frame from RGB camera
        depth = get_depth() #get a frame from depth sensor
        # frame shape: (640,480)
        frame = get_video() #get a frame from RGB camera
        depth = get_depth() #get a frame from depth sensor
        
        targetx, targety = 320, 200
        dist = how_far(depth, targety, targetx)

        cv2.rectangle(frame,(315,targety-5),(325,targety+5),(200,100,20),2)
        cv2.rectangle(depth,(315,targety-5),(325,targety+5),(200,100,20),2)

        cv2.putText(frame,str(dist),(315,230),fontFace= 3, fontScale= 0.75, color= (150,20,20),thickness= 2)
        
        cv2.imshow('RGB image',frame) #display RGB image
        cv2.imshow('Depth image',depth) #display depth image
        # print(f"far: {(how_far(depth, targety, targetx))} cm binary: {depth[targety,targetx]}")


        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()