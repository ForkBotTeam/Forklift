#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from math import tan, pi, cos, sin, atan2
from numpy.linalg import norm
from time import sleep


class wheel_odom:
    def __init__(self):
        self.rear_length = 0.18
        self.length = 0.37
        self.steering_wheel_radius = 0.1

        self.heading = 0
        self.angular_velocity = 0
        
        self.abs_orien = 0
        self.xcurrent = 0
        self.ycurrent = 0
        self.chassis_velocity = 0


        rospy.init_node("wheel_odom")
        location = rospy.Publisher("/encoder", Odometry, queue_size=10)
        rospy.Subscriber("/imu", Imu, self.imu_callback)

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.speed) # Get angular velocity of steering wheel
        rospy.Subscriber("/tf", TFMessage, self.direction) # Get heading of steering wheel

        print("wheel encoder ON")
        rate = rospy.get_param("/simulation/wheel_encoder_rate", 20) #get rate else set to default
        rate = rospy.Rate(rate)

        self.tictoc = rospy.get_time()
        self.imu_timestamp = rospy.Time(1)  # Initialize with an arbitrary time
        
        while not rospy.is_shutdown():
            self.where_am_I()
            coordinates = Odometry()
            coordinates.header.stamp = self.imu_timestamp

            coordinates.pose.pose.position.x = self.xcurrent
            coordinates.pose.pose.position.y = self.ycurrent

            coordinates.twist.twist.linear.x = self.chassis_velocity


            q = quaternion_from_euler(0, 0, self.abs_orien) 
   
            coordinates.pose.pose.orientation.z  = q[2]
            coordinates.pose.pose.orientation.w  = q[3]

            coordinates.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

            coordinates.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
            
            location.publish(coordinates)

            rate.sleep()

    def imu_callback(self, imu_msg):
        #Callback function to update the timestamp from the IMU
        self.imu_timestamp = imu_msg.header.stamp    

    def speed(self, data):
        # Get angular velocity of steering wheel
        vx = data.twist[-1].angular.x
        vy = data.twist[-1].angular.y
        self.angular_velocity = norm([vx, vy])

    def direction(self,data):
        # Get heading of steering wheel
        if len(data.transforms) >= 3:
            orientations =  data.transforms[-2].transform.rotation
            orientations = [orientations.x ,orientations.y, orientations.z, orientations.w] 
            orientations = list(euler_from_quaternion(orientations))
            self.heading = orientations[2]

    def where_am_I(self):
        # Calculations done at chassis center of mass

        velocity = self.angular_velocity * self.steering_wheel_radius
        rotation_radius_rear = self.length / (tan(self.heading)+0.0000001)
        beta = atan2(self.rear_length*tan(self.heading) , self.length)
        rotation_radius = rotation_radius_rear/cos(beta)

        abs_orien_dot = velocity / rotation_radius
        vx = velocity*cos(self.abs_orien + beta)
        vy = velocity*sin(self.abs_orien + beta)

        sampling_rate = rospy.get_time() - self.tictoc
        self.tictoc = rospy.get_time()

        self.abs_orien -= abs_orien_dot*sampling_rate 

        self.xcurrent += vx*sampling_rate
        self.ycurrent += vy*sampling_rate

        self.chassis_velocity = norm([vx,vy])

        # print(f"x: {self.xcurrent:.2f} y: {self.ycurrent:.2f} heading: {self.heading*180/pi:3.2f} orien: {(self.abs_orien*180/pi)%360:3.2f}   ", end = "\r")


if __name__== "__main__":
    sleep(12)
    try:
        test = wheel_odom()
    except rospy.ROSInterruptException:
        pass


