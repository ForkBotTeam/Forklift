#!/usr/bin/env python3

import rospy
from math import tan, pi
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from time import sleep


class middle_man:        
    def __init__(self):
        rospy.init_node('tester', anonymous=True)
        print("Middle Man ON")
        self.steer = rospy.Publisher('/simulation/steering_axel/command', Float64, queue_size=10)
        self.drive = rospy.Publisher('/simulation/drive_wheels/command', Float64MultiArray, queue_size=10 )
    
        rospy.Subscriber('/external_control', Twist, self.external_control_feedback)
        rate = rospy.get_param("/simulation/middle_man_rate", 50) #get rate else set to default
        rate = rospy.Rate(rate)
        
        # Parameters
        self.fork_width = 0.8
        self.fork_length = 0.5
        self.RoR = 0

        self.manual_control = 1
        self.steer_angle = 0
        self.left = 0
        self.right = 0
        
        while not rospy.is_shutdown():
            try:
                velocity, self.steer_angle = list(map(float, input("insert velocity & steer angle(degree): \n").split()))
                self.steer_angle *= pi/180
                self.modification(velocity, self.steer_angle)

                speeds = Float64MultiArray()
                speeds.data = [self.right, self.left]
                self.steer.publish(self.steer_angle)
                self.drive.publish(speeds)
            except:
                print("wrong format!")

            # print(f"left: {round(left,1)} right: {round(right,1)} steer: {steer_angle}")
            rate.sleep()

    def external_control_feedback(self, data):
        self.manual_control = 0
        self.steer_angle = data.angular.z
        linear_velocity = data.linear.x/0.125 #change angular velocity to linear
        self.modification(linear_velocity, self.steer_angle)
        speeds = Float64MultiArray()
        speeds.data = [self.right, self.left]
        self.steer.publish(self.steer_angle)
        self.drive.publish(speeds)

    def modification(self, velocity, steer_angle):
        if steer_angle == 0:
            self.left = velocity
            self.right = velocity
        elif steer_angle < 0:
            RoR = self.fork_length/tan(steer_angle*-1) - self.fork_width/2
            self.left = RoR/(RoR+self.fork_width/2) * velocity
            self.right = (RoR + self.fork_width)/(RoR + self.fork_width/2) * velocity
        else:
            RoR = self.fork_length/tan(steer_angle) - self.fork_width/2
            self.left = (RoR + self.fork_width)/(RoR + self.fork_width/2) * velocity
            self.right = RoR/(RoR+self.fork_width/2) * velocity


if __name__ == '__main__':
    sleep(10)
    try:
        main = middle_man()
    except rospy.ROSInterruptException:
        pass