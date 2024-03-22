#!/usr/bin/env python3

import freenect
import cv2
import numpy as np
import rospy

class corner_finder():
    def __init__(self):
        rospy.init_node("watch_tower")
        rate = rospy.Rate(10)

        # Array initiation for faster operation
        self.frame = np.empty([480,640,3])
        self.quantitative_depth = np.empty([480,640,3])
        self.visual_depth = np.empty([480,640,3])
        self.frame_original = np.empty([480,640,3])

        # Fake YOLO, create a slice aimed at the box
        posx = slice(130,300)
        posy = slice(280,480)
        SCx = int((posx.stop+posx.start)/2) #slice_centerx
        SCy = int((posy.stop+posy.start)/2) #slice_centery
        refx = slice((SCx-10),(SCx+10))
        refy = slice((SCy-10),(SCy+10))

        # A Valley-shaped array used as a range of weights to properly isolate an object
        decreasing_array = np.arange((posy.stop-posy.start+6)/2, 3, -1)
        both_halves = np.hstack([decreasing_array, decreasing_array[::-1]])
        self.valley = np.tile(both_halves, (posx.stop-posx.start, 1))
       
        while 1:
            self.frame = self.get_video() #get a frame from RGB camera
            self.frame_original = np.copy(self.frame)
            self.quantitative_depth, self.visual_depth = self.get_depth() 
            #>>> Visual depth: normalized for displaying. 
            #>>> Quantitative depth: has correct distacne values in mm 


            center_distance = np.mean(self.quantitative_depth[refx,refy])

            mask = self.isolate(self.quantitative_depth[posx, posy], center_distance)
            gray = cv2.cvtColor(self.frame_original[posx, posy],cv2.COLOR_BGR2GRAY)
            self.display(gray*mask, "isolated", 3)

            self.corner(self.frame_original[posx,posy], mask, posx.start, posy.start) 

            i ,j = 336,190
            # self.real_coordinates(i, j, (posx.stop-posx.start), (posy.stop-posy.start))
            self.real_coordinates(i, j, 640, 480)
            
            cv2.circle(self.frame, (i, j), 2, (200,100,20),2)



            self.display(self.frame_original[posx,posy], "original", 0)
            self.display(self.frame, "cornered", 1)
            self.display(self.visual_depth, "depth map", 2)

            k = cv2.waitKey(5) & 0xFF # quit program when 'esc' key is pressed
            if k == 27:
                # cv2.imwrite('Documents/fork/src/forklift_vision/images/depth_pic.jpg', self.visual_depth)
                break
            rate.sleep()

        freenect.sync_stop()
        cv2.destroyAllWindows()

    def how_far(self, x, y, **kwargs): # "depth" are kinect depth frame, x,y are pixel locations
        # Remember x-axis translates to indexing of coloumns while y-axis is indexing of rows!
        dist = str(self.quantitative_depth[y,x]/1000)
        if "print" in kwargs:
            cv2.putText(kwargs["print"], dist,(x+5, y+5 ),fontFace= 3, fontScale= 0.75, color= (150,20,20),thickness= 2)    
        return dist

    def isolate(self, depth, ref):
        margin = 500
        # depth = depth.transpose()
        # img = np.array([(depth < ref+margin) & (depth > ref-margin)]*3).transpose() * img

        # img = ((depth < ref+margin) & (depth > ref-margin)) 
        # mask = np.ones(img.shape, dtype=np.uint8)
        # mask = img*mask
        pos_ref = ref + self.valley*8
        neg_ref = ref - self.valley*8
        condition = ((depth < pos_ref) & (depth > neg_ref))
        mask = np.ones(depth.shape, dtype=np.uint8)*condition
        
        return mask

    def corner(self, target, mask, xbound, ybound):
        #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        # EDGE DETECTION USING HOUGH-TRANSFORM METHOD  
        mask = cv2.medianBlur(mask, 15)
        gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY) 
        gray = cv2.medianBlur(gray, 5)

        gray *= mask
        gray = np.uint8(gray)

        self.display(gray,"blurred", 4)

        edges = cv2.Canny(gray, 100, 150, apertureSize=3)
        self.display(edges, "canny",5)        
        lines = cv2.HoughLinesP(edges, 1, np.pi/180,50, None, 100,20)
        
        try:
            # The below for loop runs till r and theta values are in the range of the 2d array
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(self.frame, (l[0]+ybound, l[1]+xbound), (l[2]+ybound, l[3]+xbound), (0, 0, 255), 2, cv2.LINE_AA)
        except :
            # print("There are no lines BRO!")
            pass

    def real_coordinates(self,i, j, w, h):
        # Coordinate from camera's prespective
        # (X)-----------> (x)
        #  |
        #  |
        #  |
        #  |
        #  V
        #  (y)
        #z-1070 y-110 x-0
        minDistance = -10
        scaleFactor = 0.0021
        z = self.quantitative_depth[j, i]

        if isinstance(i, np.ndarray):
            

        x = (i - w / 2) * (z + minDistance) * scaleFactor
        y = (j - h / 2) * (z + minDistance) * scaleFactor
        print(f"x: {x:.2f} y: {y:.2f} z: {z:.2f}",end="\r")

    def get_video(self): #function to get RGB image from kinect
        d = freenect.sync_get_video()
        if d is not None:
            array, _ = d
            array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
        else:
            print("failed...retrying")
            return Exception
        return array
    
    def get_depth(self): #function to get depth image from kinect
        d = freenect.sync_get_depth(format = freenect.DEPTH_REGISTERED)

        if d is not None:
            depth, _ = d
            visualize_array = cv2.normalize(depth,dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
            visualize_array = visualize_array.astype(np.uint8)
        else:
            print("failed...retrying")
            return Exception
        return depth, visualize_array

    def display(self,image, window_name, window_number):
        cv2.namedWindow(window_name) # Create a named window
        if window_number > 2:
            cv2.moveWindow(window_name, 650*(window_number-3),520) # Position window
        else:
            cv2.moveWindow(window_name, 650*window_number,0) # Position window
        cv2.imshow(window_name,image) # Display image


if __name__ == "__main__":
    # try:
    #     camera = corner_finder()
    # except:
    #     print(f"There is some error man! DAMN \n {Exception}") 
    camera = corner_finder()

