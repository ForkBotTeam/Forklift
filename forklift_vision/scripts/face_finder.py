#!/usr/bin/env python3

import freenect
import cv2
import numpy as np
import rospy
from scipy.ndimage import generic_filter

class corner_finder():
    def __init__(self):
        rospy.init_node("watch_tower")
        rate = rospy.Rate(10)
        self.frame = np.empty([480,640,3])
        self.quantitative_depth = np.empty([480,640,3])
        self.visual_depth = np.empty([480,640,3])
        self.frame_original = np.empty([480,640,3])

        posx = slice(140,300)
        posy = slice(280,480)
        
        while 1:

            self.frame = self.get_video() #get a frame from RGB camera
            self.frame_original = np.copy(self.frame)

            self.quantitative_depth, self.visual_depth = self.get_depth() 
            #>>> Visual depth: normalized for displaying. 
            #>>> Quantitative depth: has correct distacne values in mm 

            refrence_distance = np.mean(self.quantitative_depth[190:230, 360:400])
            target = self.isolate(self.visual_depth[posx, posy], self.quantitative_depth[posx, posy], refrence_distance)
            self.display(target,"isolated",3)
            self.corner(target, posx.start, posy.start)

            gray = cv2.cvtColor(self.frame_original[posx,posy], cv2.COLOR_BGR2GRAY)/10 * (target*10 + 1)
            gray = np.uint8(gray)
            self.display(gray,"moded",4)
            edges = cv2.Canny(gray, 220, 255, apertureSize=5,)
            self.display(edges,"edges_moded",5)
            self.corner(target, posx.start, posy.start) 


            self.display(self.frame_original[posx, posy], "original", 0)
            self.display(self.frame, "cornered", 1)
            self.display(self.visual_depth, "depth map", 2)


            gray = cv2.cvtColor(self.frame_original[posx,posy], cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 250, apertureSize=5)
            self.display(edges,"normal edges",6)


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

    def isolate(self, img, depth, ref):
        margin = 500
        # depth = depth.transpose()
        # # cond = np.std(depth)

        # img = np.array([(depth < ref+margin) & (depth > ref-margin)]*3).transpose() * img
        img = ((depth < ref+margin) & (depth > ref-margin)) 
        mask = np.ones(img.shape, dtype=np.uint8)
        mask = img*mask
        return mask

    def corner(self, target, xbound, ybound):
        # median_kernal = 7

        # gray_img = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
        # filter = cv2.medianBlur(gray_img,median_kernal)
    
        # canny = cv2.Canny(filter, 150, 200, apertureSize=5)
        # cv2.imshow("canny",canny)

        # #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        # # CORNER DETECTION USING SHI-TOMASI METHOD  
        # no_of_corners = 10
        # corners = cv2.goodFeaturesToTrack(canny, no_of_corners, 0.02, 50  ) 

        # # convert corners values to integer So that we will be able to draw circles on them 
        # corners = np.intp(corners)
        # for i in corners: 
        #     x, y = i.ravel() 
        #     cv2.circle(self.frame, (ybound + x, xbound + y), 3, (255, 0, 0), -1)
        #     self.how_far(x+xbound, y + ybound , print= self.frame)


        #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        # EDGE DETECTION USING HOUGH-TRANSFORM METHOD  
        # gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
        gray = target
        median_kernal = 9
        sigma = 10

        gray = cv2.medianBlur(gray,median_kernal)

        # gray = cv2.GaussianBlur(gray, ksize=(7,7),sigmaX=sigma, sigmaY=sigma)
        # edges = cv2.Canny(gray, 100, 250, apertureSize=3)
        # self.display(edges, "canny",4)
        
        lines = cv2.HoughLines(gray, 1, np.pi/180, 150)
        
        try:
            # The below for loop runs till r and theta values are in the range of the 2d array
            for r_theta in lines:
                arr = np.array(r_theta[0], dtype=np.float64)
                r, theta = arr
                # Stores the value of cos(theta) in a
                a = np.cos(theta)
            
                # Stores the value of sin(theta) in b
                b = np.sin(theta)
            
                # x0 stores the value rcos(theta)
                x0 = a*r
            
                # y0 stores the value rsin(theta)
                y0 = b*r
            
                # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
                x1 = int(x0 + 1000*(-b)) + ybound
            
                # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
                y1 = int(y0 + 1000*(a)) + xbound
            
                # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
                x2 = int(x0 - 1000*(-b)) + ybound
            
                # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
                y2 = int(y0 - 1000*(a)) + xbound
            
                # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
                # (0,0,255) denotes the colour of the line to be
                # drawn. In this case, it is red.
                cv2.line(self.frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        except :
            print("I found NOTHING")

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
            cv2.moveWindow(window_name, 650*(window_number-3),500) # Position window
        else:
            cv2.moveWindow(window_name, 650*window_number,0) # Position window
        cv2.imshow(window_name,image) # Display image


if __name__ == "__main__":
    # try:
    #     camera = corner_finder()
    # except:
    #     print(f"There is some error man! DAMN \n {Exception}") 
    camera = corner_finder()

