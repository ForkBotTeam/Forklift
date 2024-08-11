#! /usr/bin/env python2.7

import numpy as np
from numpy import array, pi
import rospy
import math
from time import time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8 , Float64, Bool
from forklift_live.msg import ard_odom

from tf.transformations import euler_from_quaternion


#####################################################################
####          Skeleton code for controller node                  ####
####    Feel free to change/add functions as you wish            ####
####    You can also change the arguments of the functions       ####
####    Note: Don't alter the node name                          ####
#####################################################################




class Controller():
    def __init__(self):

        self.dt = 0.1
        self.setpointVelocity = 2.0
        self.currentVelocity = 0.0
        self.lookaheadDistance = 0.65
        self.brakingMargin = 0.4

        self.L = 0.5

        self.my_pos_x = 0
        self.my_pos_y = 0
        self.my_orien = 0
        self.my_wheel_heading = 0

        # PID VARIABLES
        self.drive_PID_Selector = 1
        self.kp_drive, self.ki_drive, self.kd_drive = [2.0, 0, 0]
        self.speed_turning_ratio = 1.0
 
        self.steer_PID_Selector = 2
        self.angleLimit = 35*pi/180
        self.kp_steer, self.ki_steer, self.kd_steer = [200 , 50, 0.0]
        #self.kp_steer, self.ki_steer, self.kd_steer = [400 , 100, 0.0]
        self.ang = 0
        self.pwm_limit = 80
        self.testing = False

        self.prev_angle =0.0
        self.error = 0.0
        self.error_dot = 0.0
        self.error_integral_steer = 0.0
        self.error_integral_drive = 0.0
        self.error_previous = 0.0
        
        self.reverse = 1
        self.reverse_margin = pi/2

        self.initialx = 0
        self.initialy = 0
        self.first_time = True
        self.backdrive = False
        # self.backDriveMargin = 0.2

        self.back_margin = pi/2
        self.moving_back = 0
        self.parking_alligned = 0

        self.target_orientations = []
        self.path = []
        self.lastPath = []
        self.targetIndex = 0
        self.pathReceived = False

        self.stop = False
        self.brake = False

        # IDs indicating whther i am arrived or not
        # the message is published from the controller_live node
        # we indicate the arrival when the controller
        # enters the braking condition
        self.ARRIVED_ID = ord( 'A' )
        self.NOT_ARRIVED_ID = ord( 'x' )
        self.arrived_obj = Int8()

        # INIT Nodes
        rospy.init_node('controller2')
        print("controller ON")

        # INIT PUBLISHERS
        self.updateControl = rospy.Publisher('/controller_live', Twist, queue_size=10)
        self.arrived_publisher = rospy.Publisher('/arrived',Int8, queue_size=1)
        
        # INIT SUBSCRIBERS
        rospy.Subscriber('/encoder', Odometry, self.velocityCallback)
        #rospy.Subscriber('/robot_pose_ekf/odom_combined' ,PoseWithCovarianceStamped, self.positionCallback)
        rospy.Subscriber('/amcl_pose' ,PoseWithCovarianceStamped, self.positionCallback)
        rospy.Subscriber('/sw_angle_topic', Float64, self.wheel_heading) # Get angular velocity of steering wheel
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.pathCallback)
        rospy.Subscriber('/backDrive',Bool , self.backDriveCallback)

        rospy.Subscriber('/Estop',Int8,  self.EstopCallback)
        rospy.Subscriber('/dummy_input_angle',Int8,  self.dummy_angle_cbk)

        # rate = rospy.get_param("/simulation/controller_rate", 50) #get rate else set to default
        rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():
            self.executer()
        # rospy.spin()
            rate.sleep()

    def dummy_angle_cbk(self,data):
        self.ang = data.data
	
    def velocityCallback(self, data):
        ### Call back function for state message ###
        self.currentVelocity = data.twist.twist.linear.x * self.reverse

    def positionCallback(self, data):
        self.my_pos_x = data.pose.pose.position.x
        self.my_pos_y = data.pose.pose.position.y

        orientations = data.pose.pose.orientation
        orientations = [orientations.x ,orientations.y, orientations.z, orientations.w] 
        orientations = list(euler_from_quaternion(orientations))

        self.my_orien = orientations[2]

    def EstopCallback(self,msg):
        if msg.data == 1 :
            self.stop = True

        elif msg.data == 0:
            self.stop = False

    def pathCallback(self, mypath):
        ### Call back function for path message ###

        newPath = []
        for pose in mypath.poses:
            z = list(euler_from_quaternion([0,0,pose.pose.orientation.z,pose.pose.orientation.w]))[2]
            self.target_orientations.append(z)
            newPath.append([pose.pose.position.x, pose.pose.position.y])

        if newPath != self.path and self.moving_back == 0:
            self.pathReceived = False

            del self.path [:]
	        #self.path.clear()
            self.path = newPath
            self.targetIndex = 0
            self.stop=False
            self.pathReceived = True
            
        elif newPath == None:
            self.pathReceived = False

    def backDriveCallback(self,data):
        if self.first_time:
            self.initialx = self.my_pos_x
            self.initialy = self.my_pos_y
            
            self.first_time = False
            self.backdrive = True
 
    def backDrive(self):
        distance = np.sqrt((self.initialx-self.my_pos_x)**2 + (self.initialy-self.my_pos_y)**2)
        if distance > self.backDriveMargin:
            self.backdrive = False
            self.first_time = True
            print("Distance Done")


    def executer(self):
        if self.pathReceived == True:
            update = Twist()
            myx =  self.my_pos_x
            myy =  self.my_pos_y
            theta =  self.my_orien
            
            dist_to_goal, self.targetIndex = self.searchTargetPoint()
            target_x, target_y = self.path[self.targetIndex]

            # Back driving
            # path_angle = np.arctan2((target_y - myy), (target_x - myx)) # Orien of next point relative to fork
            # included_angle = abs(path_angle - theta )
            # if included_angle >pi:
            #     included_angle -=pi*2
            # included_angle = abs(included_angle)


            # if abs (included_angle > self.reverse_margin) :#> 10*pi/180 :
            #     # self.reverse_margin = 85*pi/180  
            #     self.reverse = -1
            #     print("I am In BACK")
            # else:
            #     # self.reverse_margin = pi/2 
            #     self.reverse = 1
            #     print("I am In FRONT")
            
            
            if self.backdrive:
                print("I am in Backdrive")
                self.backDrive()
                if abs(self.my_wheel_heading*180/pi) < 5.0:
                    update.linear.x = self.speed_controller()*-255 
                else:
                    update.linear.x =0.0
                update.angular.z = self.PID(self.steer_PID_Selector,self.kp_steer, self.ki_steer*2.0, self.kd_steer, 0, self.my_wheel_heading, clip = self.pwm_limit)
                print('current heading %.2f target heading %.2f action %.2f' % (self.my_wheel_heading*180/pi, 0*180/pi, update.angular.z))

                
            elif dist_to_goal > 2 and not (self.stop or self.brake):
                self.parking_alligned = 0
                self.action = self.speed_controller()
                self.steering_angle = self.purePursuit(myx, myy, theta, target_x, target_y)

            elif dist_to_goal <= 2 and not (self.stop or self.brake):
                self.action, self.steering_angle = self.parking(myx, myy, theta, self.target_orientations[-1])

            elif self.stop or self.brake:
                update.linear.x = 0.0
                self.steering_angle = 0.0

            path_angle = np.arctan2((target_y - myy), (target_x - myx)) # Orien of next point relative to fork
            included_angle = abs(path_angle - theta )
            

            if included_angle > self.back_margin and self.brake == False:
                self.moving_back = 1
                self.back_margin = 85*pi/180
                self.action *= -1
                self.steering_angle = -(30*pi/180)*(abs(self.steering_angle)/(self.steering_angle+0.00001))   
            else:
                self.moving_back = 0
                self.back_margin = pi/2


            self.updateControl.publish(update)

    def wheel_heading(self, data):
        if self.backdrive:
            self.my_wheel_heading = data.data*(pi/180) 
        else:
            self.my_wheel_heading = data.data*(pi/180) #* self.reverse


    def speed_controller(self):
        
        ###            Implement PID controller here                   ###
        #          Take as an input the current state of the car         #
        #                   Return acceleration                          #

        if self.stop :
            return -1.0
        elif self.brake:
            print("arrived")
            return 0.0
        if abs (self.my_wheel_heading*180/pi) > 20.0 :
            self.setpointVelocity = self.setpointVelocity * self.speed_turning_ratio 
        
        action = self.PID(self.drive_PID_Selector,self.kp_drive, self.ki_drive, self.kd_drive, self.setpointVelocity, self.currentVelocity, clip = 1)

        return action


    def purePursuit(self, myx, myy, theta, target_x, target_y):
        
        ###              Implement pure pursuit controller here               ###
        #            Take as an input the current state of the car,             #
        #                     waypoints & targetIndex                           #
        #                      Return steering angle                            #        
        if self.stop or self.brake:
            return 0.0

        alpha = np.arctan2(target_y - myy, target_x - myx) - theta
        steering_angle = np.arctan2(2*self.L*np.sin(alpha) , self.lookaheadDistance)
        steering_angle = np.clip(steering_angle, -self.angleLimit, self.angleLimit)*-1

        if self.testing:
            steering_angle=(pi/180) * self.ang


        action = self.PID(self.steer_PID_Selector,self.kp_steer, self.ki_steer, self.kd_steer, steering_angle, self.my_wheel_heading, clip = self.pwm_limit)

        if abs(self.my_wheel_heading) > (self.angleLimit*pi/180):
            action =action*1
     
        print('%.2f xcurrent %.2f ycurrent %.2f targetx %.2f targety %.2f current heading %.2f target heading %.2f action %.2f' % (time(), myx, myy, target_x, target_y, self.my_wheel_heading*180/pi, steering_angle*180/pi, action))
        
        return action
    

    def parking(self, myx, myy, mytheta, goal_orien):
        self.targetIndex = 0
        dist_to_goal = 0
        if self.parking_alligned == 0:
            action = 0.3

            path = np.array(self.path[-1:-4:-1])
            r = np.array([0, 0.5, 1]) # Parking range for last 3 path points
            path = np.array([path[:,0] + r*np.cos(goal_orien), path[:,1] + r*np.sin(goal_orien)]).transpose()   
            print(path)
            self.targetIndex, dist_to_goal = self.searchTargetPoint(path)

            targetPoint = path[self.targetIndex]
            target_x, target_y = targetPoint 

            steering_angle = self.purePursuit(myx, myy, mytheta, target_x, target_y)

        if dist_to_goal < 0.5:
            action = -0.3

            self.parking_alligned = 1
            targetIndex, dist_to_goal = self.searchTargetPoint(self.path)
            steering_angle = 0

        return action, steering_angle
   

    def PID(self,selector ,kp, ki, kd, setpoint, feedback, **kwargs):
        error = setpoint - feedback
        max_error = 160*pi/180

        error_dot = (error - self.error_previous)/self.dt

        self.error_previous = error

        if selector == self.steer_PID_Selector:

            error = error/max_error
            error_dot = error_dot
            self.error_integral_steer += error*self.dt

            if self.prev_angle != setpoint or abs(error) > 30.0/160 :
                self.error_integral_steer =0
                #print(abs(error), 30/160)
                

            #Cap integral error at certain value
            if abs(self.error_integral_steer) > 0.4:
                # print("HERE")
                self.error_integral_steer = 0.1*(self.error_integral_steer/self.error_integral_steer)

            self.prev_angle = setpoint
            action = kp*error + ki*self.error_integral_steer + kd*error_dot

        else:
            error = abs(error/max_error)
            error_dot = abs(error_dot)
            
            self.error_integral_drive += error*self.dt
            action = kp*error + ki*self.error_integral_drive + kd*error_dot
    
        try:
            if kwargs['clip']:
                action = np.clip(action, -kwargs["clip"], kwargs["clip"])
        except:
            pass


        if  selector == self.drive_PID_Selector :
            return action * self.setpointVelocity / abs(self.setpointVelocity)
        
        if abs(error) > (2/160) :
            return action
        else:
            self.error_integral_steer = 0
            self.error_integral_drive = 0
            return 0    
        

    def searchTargetPoint(self, path):
        ###           Search for target point in the given path               ###
        #       Take as an input the current state of the car & waypoints       #
        #                   Return index of target point                        #

        distances = np.linalg.norm(array(path[self.targetIndex:]) - array([self.my_pos_x, self.my_pos_y]).transpose(), axis=1)
        min_index = np.argmin(distances)

        print("distance now: %.2f" %(distances[-1]))
        self.brakingMargin = self.braking(distances[-1], self.brakingMargin)

        RemainingPathPoints = len(distances)
        for i in range(min_index, RemainingPathPoints):
            if distances[i] >= self.lookaheadDistance:
                return i + self.targetIndex, distances[-1]
            elif distances[-1] < self.lookaheadDistance:
                return RemainingPathPoints + self.targetIndex - 1, distances[-1]
        return min_index + self.targetIndex, distances[-1]
        

    def braking(self, current_distance, margin):
        if current_distance < margin:
            self.brake = True
            self.arrived_obj.data = self.ARRIVED_ID
            self.arrived_publisher.publish(self.arrived_obj)
            self.ang = 0
            return 0.5
            
        else:
            self.arrived_obj.data = self.NOT_ARRIVED_ID
            self.arrived_publisher.publish(self.arrived_obj)
            self.brake = False
            self.testing = False
            return 0.3

if __name__ == '__main__':
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass
        
    


    
    
    
