#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import geometry_msgs.msg

import time


br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

def main():
    global br, t
    rospy.init_node('dynamic_tf2_broadcaster')

    print("transformer ON")
   
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"

    while not rospy.is_shutdown():
        rospy.Subscriber("/encoder", Odometry, transformer)
        rospy.spin()

def transformer(data):
    global br, t
    now = rospy.Time.now()  

    t.header.stamp = now
    t.transform.translation.x = data.pose.pose.position.x 
    t.transform.translation.y = data.pose.pose.position.y
    
    t.transform.rotation.z = data.pose.pose.orientation.z
    t.transform.rotation.w = data.pose.pose.orientation.w
    
    br.sendTransform(t)

if __name__== "__main__":
    time.sleep(9)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    