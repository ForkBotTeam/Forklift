#!/usr/bin/env python3

import freenect
import cv2
import numpy as np
import rospy
from itertools import combinations

class corner_finder():
    def __init__(self):
        rospy.init_node("watch_tower")
        rate = rospy.Rate(10)

        # Array initiation for faster operation
        self.frame = np.empty([480,640,3], dtype= np.uint8)
        self.quantitative_depth = np.empty([480,640,3], dtype= np.uint16)
        self.visual_depth = np.empty([480,640,3], dtype= np.uint8)
        self.frame_original = np.empty([480,640,3], dtype= np.uint8)

        # Fake YOLO, create a slice aimed at the box
        posx = slice(170,400)
        posy = slice(214,390)

        self.rangex = posx.stop - posx.start
        self.rangey = posy.stop - posy.start
        SCx = int((posx.stop+posx.start)/2) #slice_centerx
        SCy = int((posy.stop+posy.start)/2) #slice_centery
        refx = slice((SCx-15),(SCx+15))
        refy = slice((SCy-15),(SCy+15))

        # A Valley-shaped array used as a range of weights to properly isolate an object
        decreasing_array = np.arange((posy.stop-posy.start+6)/2, 3, -1, dtype= np.uint16)
        both_halves = np.hstack([decreasing_array, decreasing_array[::-1]], dtype= np.uint16)
        self.valley = np.tile(both_halves, (posx.stop-posx.start, 1))

        # Others
        self.renew = 0
        self.prev_lines = 0

        self.box_lines = None
        tictoc = rospy.get_time()
        while 1:
            #>>> Visual depth: normalized for displaying. 
            #>>> Quantitative depth: has correct distacne values in mm 
            self.frame = self.get_video() #get a frame from RGB camera
            self.frame_original = np.copy(self.frame)
            self.quantitative_depth, self.visual_depth = self.get_depth() 


            center_distance = np.mean(self.quantitative_depth[refx,refy])
            mask = self.isolate(self.quantitative_depth[posx, posy], center_distance)
            self.detect_edge(self.frame_original[posx,posy], mask, posx.start, posy.start) 
            real_lines = None

            if self.box_lines is not None:
                x1, y1, z1 = self.real_coordinates(self.box_lines[:,0:1], self.box_lines[:,1:2], 640, 480)
                x2, y2, z2 = self.real_coordinates(self.box_lines[:,2:3], self.box_lines[:,3:4], 640, 480)

                real_lines = np.hstack([x1,y1,z1,x2,y2,z2])

            self.face_finder(real_lines)


            self.display(self.frame_original[posx,posy], "original", 0)
            self.display(self.frame, "cornered", 1)
            self.display(self.visual_depth, "depth map", 2)

            k = cv2.waitKey(5) & 0xFF # quit program when 'esc' key is pressed
            if k == 27:
                # cv2.imwrite('Documents/fork/src/forklift_vision/images/depth_pic.jpg', self.visual_depth)
                break
            print(rospy.get_time()-tictoc)
            rate.sleep()
            tictoc = rospy.get_time()


        freenect.sync_stop()
        cv2.destroyAllWindows()

    def face_finder(self,lines):
        if lines is not None:
            x1 = lines[:,0:1]
            y1 = lines[:,1:2]
            z1 = lines[:,2:3]
            
            x2 = lines[:,3:4]   
            y2 = lines[:,4:5]
            z2 = lines[:,5:6]

            line_vectors = np.hstack([x2-x1 , y2-y1 , z2-z1])
            no_of_lines = line_vectors.shape[0]

            # We want to find all possible planes between these vectors
            if no_of_lines > 1:
                mixat = np.array(list(combinations(line_vectors,2)))
                planes = np.cross(mixat[:,0],mixat[:,1])

                # print(f"no. of planes: {planes.shape[0]}\n")
                plane_norm = np.linalg.norm(planes, axis=1)
                # print(plane_norm)
                plane_angles = np.arccos((planes[:,2])/plane_norm)
                # print(plane_angles*180/np.pi)  

            else:
                pass
                # print("there are no planes")

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
            for a in range(0, i.shape[0]):
                self.how_far(i[a,0], j[a,0])

            x = (i - w / 2) * (z + minDistance) * scaleFactor
            y = (j - h / 2) * (z + minDistance) * scaleFactor
        else:
            x = (i - w / 2) * (z + minDistance) * scaleFactor
            y = (j - h / 2) * (z + minDistance) * scaleFactor
            self.how_far(i, j, print = self.frame)

        return x,y,z
    
    def how_far(self, x, y, **kwargs): # "depth" are kinect depth frame, x,y are pixel locations
        # Remember x-axis translates to indexing of coloumns while y-axis is indexing of rows!
        dist = str(self.quantitative_depth[y,x]/1000)

        if "print" in kwargs:
            cv2.putText(kwargs["print"], dist,  (int(x+5),int(y+5)),fontFace= 3, fontScale= 0.75, color= (150,20,20),thickness= 2)    
        return dist

    def detect_edge(self, target, mask, xbound, ybound):
        #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        # EDGE DETECTION USING HOUGH-TRANSFORM METHOD  
        target *= mask
        gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY) 
        gray = cv2.medianBlur(gray, 5)

        # gray *= mask
        # gray = np.uint8(gray)

        self.display(gray,"blurred", 4)

        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        self.display(edges, "canny",5)  

#########################################################

        voting = int(np.min([self.rangex, self.rangey])/2)-20  # no of voting points = min edge of the bounding box - a margin         
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, voting, None, voting, 20)
        
        lsd = cv2.createLineSegmentDetector(0)
        lsd_lines = lsd.detect(gray)[0][:,0] + np.array([ybound, xbound, ybound, xbound], dtype=np.float32)
        
        tst_img2 = np.copy(self.frame)
        tst_img2 = lsd.drawSegments(tst_img2, lsd_lines)
        self.display(tst_img2, "lsd", 3)

        if lines is not None:        
            self.box_lines = lines[:,0] + np.array([ybound, xbound, ybound, xbound])
            # print(lines)
            # The below for loop runs till r and theta values are in the range of the 2d array
            for i in range(0, len(lines)):
                # l = lines[i][0]
                cv2.line(self.frame, (self.box_lines[i,0], self.box_lines[i,1]), (self.box_lines[i,2], self.box_lines[i,3]), (0, 0, 255), 2, cv2.LINE_AA)


    def isolate(self, depth, ref):
        # margin = 500
        # condition = (depth < ref+margin) & (depth > ref-margin)
        # mask = np.ones(depth.shape, dtype=np.uint8)*condition

        pos_ref = ref + self.valley*8
        neg_ref = ref - self.valley*8
        condition = ((depth < pos_ref) & (depth > neg_ref))
        mask = np.ones(depth.shape, dtype=np.uint8)*condition
        smoother = int(np.max([self.rangex, self.rangey])/45)*2 + 3
        mask = cv2.medianBlur(mask, smoother)
        mask = np.array([mask.transpose()]*3).transpose()
        
        return mask

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
            cv2.medianBlur(depth, 5, dst=depth)
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

