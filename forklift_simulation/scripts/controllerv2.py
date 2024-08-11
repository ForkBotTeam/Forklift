#!/usr/bin/env python3

import numpy as np
from numpy import array, pi
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import  Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8

from tf.transformations import euler_from_quaternion

from time import sleep,time

#####################################################################
####          Skeleton code for controller node                  ####
####    Feel free to change/add functions as you wish            ####
####    You can also change the arguments of the functions       ####
####    Note: Don't alter the node name                          ####
#####################################################################

class Controller():
    def __init__(self):

        self.setpointVelocity = 1.0
        self.currentVelocity = 0.0
        self.lookaheadDistance = 1
        self.brakeMargin = 0.3

        self.L = 0.5
        self.angleLimit = 90*pi/180

        self.action = 0
        self.steering_angle = 0

        self.pose= [0,0,0]

        # PID VARIABLES
        self.kp_drive, self.ki_drive, self.kd_drive = [1, 0.5, 0]
        self.kp_steer, self.ki_steer, self.kd_steer = [5, 2, 0]
        self.error_integral_steer = 0
        self.error_integral_drive = 0
          
        self.error = 0.0
        self.error_dot = 0.0
        self.error_integral = 0.0
        self.error_previous = 0.0
        
        # PATH VARIABLES
        self.path = array([0,0])
        self.lastPath = []
        self.targetIndex = 0
        self.pathReceived = False
        self.map_frame = "odom"
        self.target_orientations = 0
        self.back_margin = pi/2
        # distance from goal at which points are removed and custom orientation points are added
        self.remove_threshold = 2 

        self.stop = False
        self.brake = False



        # INIT Nodes
        rospy.init_node('controller')
        print("controller ON")

        # INIT PUBLISHERS
        self.nextPointPub =  rospy.Publisher('/target_point',Marker, queue_size=1)
        self.pathPub = rospy.Publisher('/controller_path', Path , queue_size=10)

        self.updateControl = rospy.Publisher('/external_control', Twist, queue_size=10)
    
        # INIT SUBSCRIBERS
        rospy.Subscriber('/encoder', Odometry, self.velocityCallback)
        rospy.Subscriber('/robot_pose_ekf/odom_combined' ,PoseWithCovarianceStamped, self.positionCallback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.pathCallback)
        rospy.Subscriber('/Estop',Int8,  self.EstopCallback)

        rate = rospy.get_param("/simulation/controller_rate", 50) #get rate else set to default
        self.dt = 1/rate 
        rate = rospy.Rate(rate)
        self.start_time = time()


        while not rospy.is_shutdown():
            self.executer()
            rate.sleep()

    def velocityCallback(self, state:Odometry):
        ### Call back function for state message ###
        self.currentVelocity = state.twist.twist.linear.x

    def positionCallback(self, data):
        orientations = data.pose.pose.orientation
        orientations = [orientations.x ,orientations.y, orientations.z, orientations.w] 
        orientations = list(euler_from_quaternion(orientations))

        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, orientations[2]]

    def EstopCallback(self,msg):
        if msg.data == 1 :
            self.stop = True

        elif msg.data == 0:
            self.stop = False

    def pathCallback(self, mypath):
        ### Call back function for path message ###

        newPath = []
        self.target_orientations = euler_from_quaternion([0,0,mypath.poses[-1].pose.orientation.z,mypath.poses[-1].pose.orientation.w])[2]

        for pose in mypath.poses:
            newPath.append([pose.pose.position.x, pose.pose.position.y])

        if len(newPath) == 0:
            self.pathReceived = False

        elif (newPath[0] != self.path[0]).all() or (newPath[-1] != self.path[-1]).all() or len(newPath) != len(self.path):
  
            self.path = array(newPath)
            self.targetIndex = 0
            self.pathReceived = True


    def executer(self):
        if self.pathReceived == True:
            update = Twist()

            myx, myy, theta = self.pose
            info = self.searchTargetPoint(self.path)
            if info is not None:
                self.targetIndex, dist_to_goal = info
                target_x, target_y = self.path[self.targetIndex]

                # Motion properties will change depending on the proximity to the goal
                if dist_to_goal > 2 and not (self.stop or self.brake):
                    self.parking_alligned = 0

                    self.action = self.speed_control()
                    self.steering_angle = self.purePursuit(myx, myy, theta, target_x, target_y)

                    path_angle = abs(np.arctan2((target_y - myy), (target_x - myx))) # Orien of next point relative to fork
                    included_angle = abs(path_angle - abs(theta)) 

                    if included_angle > self.back_margin and self.brake == False :
                        self.back_margin = 85*pi/180
                        self.action *= -1
                        self.steering_angle = -(30*pi/180)*(abs(self.steering_angle)/(self.steering_angle+0.00001))   
                    elif included_angle < self.back_margin and self.brake == False:  
                        self.back_margin = pi/2

                elif dist_to_goal <= 2 and not (self.stop or self.brake):
                    self.action, self.steering_angle = self.parking(myx, myy, theta, self.target_orientations)

                elif self.stop or self.brake:
                    self.action = 0.0
                    self.steering_angle = 0.0

                try:
                    print(f"path angle: {path_angle*180/pi:.2f} theta: {theta*180/pi:.2f} included angle: {included_angle*180/pi:.2f} y: {target_y - myy:.2f} x: {target_x - myx:.2}     ", end = '\r')
                except:
                    pass

                update.linear.x = self.action
                update.angular.z = self.steering_angle

                self.updateControl.publish(update)
                self.nextPoint()


    def nextPoint(self): #next point marker
        try:
            nextPoint = Marker()
            tx, ty = self.path[self.targetIndex]
            nextPoint.header.frame_id = self.map_frame
            nextPoint.header.stamp = rospy.Time.now()
            nextPoint.ns = "point"
            nextPoint.id = 3
            nextPoint.type = Marker.SPHERE
            nextPoint.action = Marker.ADD
            nextPoint.pose.position.x = tx
            nextPoint.pose.position.y = ty   
            nextPoint.pose.position.z = 0.1
            nextPoint.pose.orientation.x = 0.0
            nextPoint.pose.orientation.y = 0.0
            nextPoint.pose.orientation.z = 0.0
            nextPoint.pose.orientation.w = 1.0
            nextPoint.scale.x = 1.0
            nextPoint.scale.y = 1.0
            nextPoint.scale.z = 0.1
            nextPoint.color.a = 1.0
            nextPoint.color.r = 1.0
            nextPoint.color.g = 0.0
            nextPoint.color.b = 0.0
            self.nextPointPub.publish(nextPoint)
        except:
            pass      
    def speed_control(self):
        
        ###            Implement PID controller here                   ###
        #          Take as an input the current state of the car         #
        #                   Return acceleration                          #

        if self.stop :
            return -2.0
        elif self.brake:
            # print("arrived")
            return 0.0
        
        action = self.PID("driving", self.kp_drive, self.ki_drive, self.kd_drive, self.setpointVelocity, self.currentVelocity, clip = 1)

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
        steering_angle = np.clip(steering_angle, -self.angleLimit, self.angleLimit)

    
        # print(f'current ({myx:.2f}, {myy:.2f}) target: ({target_x:.2f}, {target_y:.2f}) my_orien: {theta*180/pi:.2f} steering {steering_angle*180/pi:.2f} speed {self.action:.2f}')
        return -steering_angle
    
    def parking(self, myx, myy, mytheta, goal_orien):
        if self.pathReceived == True:
            end_path = self.path[-1:-4:-1].transpose()

            distances = np.linalg.norm(self.path[:] - self.path[-1])
            #remove coordinates from path whose distance is less that of a threshold
            self.path = self.path[distances > self.remove_threshold]    
            self.path = self.path.reshape(-1,2)
            r = np.array([0, 0.5, 0.7]) # Parking range for last 3 path points
            end_path = end_path + array([r*np.cos(goal_orien), r*np.sin(goal_orien)])
        
            self.path = np.vstack([self.path, end_path.transpose()])

        try:
            self.targetIndex, dist_to_goal = self.searchTargetPoint(self.path)
            target_x, target_y = self.path[self.targetIndex]
            steering_angle = self.purePursuit(myx, myy, mytheta, target_x, target_y)
            action = 0.3
            return action, steering_angle
        except:
            return 0, 0   

   

    def PID(self,selector, kp, ki, kd, setpoint, feedback, **kwargs):
        error = setpoint - feedback

        error_dot = (error - self.error_previous)/self.dt

        self.error_previous = error

        if selector == "steering":
            self.error_integral_steer += error*self.dt
            action = kp*error + ki*self.error_integral_steer + kd*error_dot

        elif selector == "driving":
            self.error_integral_drive += error*self.dt
            action = kp*error + ki*self.error_integral_drive + kd*error_dot
    
        try:
            if kwargs['clip']:
                action = np.clip(action, -kwargs["clip"], kwargs["clip"])
        except:
            pass

        if abs(error) > (2*pi/180) or selector == "driving" :
            return action
        else:
            self.error_integral_steer = 0
            self.error_integral_drive = 0
            return 0  
    
    def searchTargetPoint(self, path):
        ###           Search for target point in the given path               ###
        #       Take as an input the current state of the car & waypoints       #
        #                   Return index of target point                        #
        distances = np.linalg.norm(path[self.targetIndex:] - array(self.pose[0:2]).transpose(), axis=1)
        if len(distances) != 0:
            if distances[-1] < self.brakeMargin:
                self.brakeMargin = 0.5
                self.brake = True
                # return self.targetIndex, distances[-1]
            else:
                self.brakeMargin = 0.3
                self.brake = False

            min_index = np.argmin(distances)
            RemainingPathPoints = len(distances)
            for i in range(min_index, RemainingPathPoints):
                if distances[i] >= self.lookaheadDistance:
                    return i + self.targetIndex, distances[-1]
                elif distances[-1] < self.lookaheadDistance:
                    return RemainingPathPoints + self.targetIndex - 1, distances[-1]
            return min_index + self.targetIndex, distances[-1]

    
if __name__ == '__main__':
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass




