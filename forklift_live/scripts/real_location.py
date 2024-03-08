#!/usr/bin/env python3

import rospy

from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import LinkStates
from math import pi
from time import sleep

first_time = 1
index = 0

def main():
    rospy.init_node("real_location")
    rospy.Subscriber("/gazebo/link_states", LinkStates, my_location)

    rate = rospy.get_param("/simulation/real_location_rate", 20) #get rate else set to default
    rate = rospy.Rate(rate)
    rate.sleep()
    rospy.spin()

def my_location(data):
    global first_time, index
    if first_time:
        for i in range(0, len(data.name)):
            if data.name[i] == "forklift::base_footprint":
                index = i
                first_time = 0
    
    x = data.pose[index].position.x
    y = data.pose[index].position.y    

    theta = data.pose[index].orientation
    theta = [theta.x, theta.y, theta.z, theta.w]
    theta = euler_from_quaternion(theta)

    # print(f"x: {x:.2f} y: {y:.2f} theta: {theta[-1]*180/pi:4.2f}   ", end="\r")

if __name__ == "__main__":
    sleep(10)
    try:
        main()
    except rospy.ROSInterruptException:
        pass

