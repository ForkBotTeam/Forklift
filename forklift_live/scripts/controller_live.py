#!/usr/bin/env python3

import numpy as np
from numpy import array, pi
import rospy
import math
from geometry_msgs.msg import  Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8
from forklift_simulation.msg import ard_odom

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
        self.setpointVelocity = 1
        self.currentVelocity = 0.0
        self.lookaheadDistance = 0.8
        self.brakingMargin = 0.3

        self.L = 0.5
        self.angleLimit = 90 *pi/180

        self.pose= [0,0,0]
        self.my_pos_x = 0
        self.my_pos_y = 0
        self.my_orien = 0
        self.my_wheel_heading = 0

        # PID VARIABLES
        self.drive_PID_Selector = 1
        self.kp_drive, self.ki_drive, self.kd_drive = [1, 0, 0]

        self.steer_PID_Selector = 2
        self.kp_steer, self.ki_steer, self.kd_steer = [4, 1.5, 0.2]
        #self.ang = -30

        self.error = 0.0
        self.error_dot = 0.0
        self.error_integral_steer = 0.0
        self.error_integral_drive = 0.0
        self.error_previous = 0.0
        
        # # PATH VARIABLES
        # self.path_x = [0.0]
        # self.path_y = [0.0]
        self.path = []

        # for i in range(0,1,1):
        #     self.path_x.append(i/10.0)
        #     self.path_y.append(0)


        # for i in range(1,15,1):
        #     i = i /10.0
        #     self.path_x.append(i)
        #     self.path_y.append((-math.sqrt(4-(i-0.1)**2))+2)

        # for i in range(10,20,1):
        #     self.path_x.append(2)
        #     self.path_y.append(i/10.0)


        # for i in range(len(self.path_x)):
        #     self.path.append([self.path_x[i],self.path_y[i]])
        print(self.path)

        #self.path = [[0,0], [0.2,0.2], [0.4,0.4], [0.6,0.6], [0.8,0.8], [1,1]]

        self.lastPath = []
        self.targetIndex = 0
        self.pathReceived = False

        self.stop = False
        self.brake = False

        self.check = True

        # INIT Nodes
        rospy.init_node('controller2')
        print("controller ON")

        # INIT PUBLISHERS
        self.updateControl = rospy.Publisher('/controller_live', Twist, queue_size=10)

        # INIT SUBSCRIBERS
        rospy.Subscriber('/encoder', Odometry, self.velocityCallback)
        #rospy.Subscriber('/robot_pose_ekf/odom_combined' ,PoseWithCovarianceStamped, self.positionCallback)
        rospy.Subscriber('/amcl_pose' ,PoseWithCovarianceStamped, self.positionCallback)
        rospy.Subscriber("/ard_odom_topic", ard_odom, self.wheel_heading) # Get angular velocity of steering wheel
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.pathCallback)

        rospy.Subscriber('/Estop',Int8,  self.EstopCallback)

        # rate = rospy.get_param("/simulation/controller_rate", 50) #get rate else set to default
        rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():
            self.timerCallback()
        # rospy.spin()
            rate.sleep()

    def velocityCallback(self, data):
        ### Call back function for state message ###
        self.currentVelocity = data.twist.twist.linear.x

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
            newPath.append([pose.pose.position.x, pose.pose.position.y])

        if newPath != self.path:
            self.pathReceived = False
            # self.stop = True
            self.path.clear()
            self.path = newPath
            self.targetIndex = 0
            self.stop=False
            self.pathReceived = True
            # print(self.path)

        elif newPath == None:
            self.pathReceived = False

    def timerCallback(self):
        if self.pathReceived == True:
            update = Twist()

            update.linear.x = self.speed_controller()
            update.angular.z = self.purePursuit()
            self.updateControl.publish(update)

    def wheel_heading(self, data):
        self.my_wheel_heading = data.angle*(pi/180)

    def speed_controller(self):
        
        ###            Implement PID controller here                   ###
        #          Take as an input the current state of the car         #
        #                   Return acceleration                          #

        if self.stop :
            return -2.0
        elif self.brake:
            # print("arrived")
            return 0.0
        
        action = self.PID(self.drive_PID_Selector,self.kp_drive, self.ki_drive, self.kd_drive, self.setpointVelocity, self.currentVelocity, clip = 1)

        return action
            
    def purePursuit(self):
        
        ###              Implement pure pursuit controller here               ###
        #            Take as an input the current state of the car,             #
        #                     waypoints & targetIndex                           #
        #                      Return steering angle                            #
        if self.stop or self.brake:
            return 0.0
        # Fail safe
        #if self.my_wheel_heading > self.steering_failsafe:
        #    return 0
        

        self.targetIndex = self.searchTargetPoint()
        targetPoint = self.path[self.targetIndex]


        myx = self.my_pos_x
        myy = self.my_pos_y
        theta = self.my_orien
        target_x = targetPoint[0]
        target_y = targetPoint[1]


        alpha = np.arctan2(target_y - myy, target_x - myx) - theta
        steering_angle = np.arctan2(2*self.L*np.sin(alpha) , self.lookaheadDistance)
        steering_angle = np.clip(steering_angle, -self.angleLimit, self.angleLimit)*-1
        # steering_angle=(pi/180) * self.ang
        # if myx > 0.5:
        #    steering_angle=(pi/180) * self.ang
        action = self.PID(self.steer_PID_Selector,self.kp_steer, self.ki_steer, self.kd_steer, steering_angle, self.my_wheel_heading, clip = self.angleLimit)

        print(f'xcurrent {myx:.2f} ycurrent {myy:.2f} targetx {target_x:.2f} targety {target_y:.2f} current heading {self.my_wheel_heading*180/pi:.2f} target heading {steering_angle*180/pi:.2f} action {(action/(pi/2))*200:.2f}')

        return action
    
    def PID(self,selector ,kp, ki, kd, setpoint, feedback, **kwargs):
        error = setpoint - feedback

        error_dot = (error - self.error_previous)/self.dt


        self.error_previous = error

        if selector == self.steer_PID_Selector:
            self.error_integral_steer += error*self.dt
            action = kp*error + ki*self.error_integral_steer + kd*error_dot

        else:
            self.error_integral_drive += error*self.dt
            action = kp*error + ki*self.error_integral_drive + kd*error_dot
    
        try:
            if kwargs['clip']:
                action = np.clip(action, -kwargs["clip"], kwargs["clip"])
        except:
            pass

        if abs(error) > (2*pi/180) or selector == self.drive_PID_Selector :
            return action
        else:
            self.error_integral_steer = 0
            self.error_integral_drive = 0
            return 0    
        
    def searchTargetPoint(self):
        ###           Search for target point in the given path               ###
        #       Take as an input the current state of the car & waypoints       #
        #                   Return index of target point                        #

        distances = np.linalg.norm(array(self.path[self.targetIndex:]) - array([self.my_pos_x, self.my_pos_y]).transpose(), axis=1)
        min_index = np.argmin(distances)

        if distances[-1] < self.brakingMargin:
            self.brake = True
        else:
            self.brake = False

        for i in range(min_index,len(distances)):
            if distances[i] >= self.lookaheadDistance:
                return i + self.targetIndex
        return min_index + self.targetIndex
        

if __name__ == '__main__':
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass
        
    


    