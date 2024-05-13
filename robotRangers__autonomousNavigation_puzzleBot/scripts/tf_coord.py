#!/usr/bin/env python3  

import rospy 

from tf.transformations import quaternion_from_euler 

import tf2_ros #ROS package to work with transformations 

from geometry_msgs.msg import TransformStamped 

import numpy as np  

from nav_msgs.msg import Odometry

 

 

class TfBroadcaster(): 

    def __init__(self): 

        rospy.init_node('tf2_broadcaster') 

        rospy.Subscriber("odom", Odometry, self.odom_cb) 

        tf_pub = tf2_ros.TransformBroadcaster()

        #Create a tf broadcaster, it will be in charge of publishing the transformations 

        # consider a broadcaster as an special publisher that works only for transformations. 

        #Create a transformation 

        # The type of message that we use to publish a transformation 

        t = TransformStamped() 

        r=rospy.Rate(50) 

        theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = [0.0, 0.0, 0.0, 0.0]

        print("Node broadcaster initialized!!") 

        print("use rviz to see the rotating transformation") 

         

        while not rospy.is_shutdown(): 

            # Fill the transformation information 

            t.header.stamp = rospy.Time.now() 

            t.header.frame_id = "base_link" 

            t.child_frame_id = "chassis" 

            t.transform.translation.x = self.x 

            t.transform.translation.y = self.y 

            t.transform.translation.z = 0.0 


            #The transformation requires the orientation as a quaternion 

            #q = quaternion_from_euler(0, 0, theta) 

            t.transform.rotation.x = self.theta[0] 

            t.transform.rotation.y = self.theta[1] 

            t.transform.rotation.z = self.theta[2] 

            t.transform.rotation.w = self.theta[3] 

            # A transformation is broadcasted instead of published 

            tf_pub.sendTransform(t) #broadcast the transformation 

            r.sleep() 


    def odom_cb(self, msg): 
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = [[msg.pose.pose.orientation.x],
                      [msg.pose.pose.orientation.y],
                      [msg.pose.pose.orientation.z],
                      [msg.pose.pose.orientation.w]]

if __name__ == '__main__': 

    TfBroadcaster() 