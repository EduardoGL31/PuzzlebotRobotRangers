#!/usr/bin/env python3 

import rospy    
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from tf.transformations import quaternion_from_euler 
import numpy as np 
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_msgs.msg import TFMessage
 
#This class will do the following: 
#   subscribe to /wr and /wl
#   publish to the /odom topic

class OdomClass():  
    def __init__(self):  
        # first thing, init a node! 
        rospy.init_node('localisation') 
        ###******* INIT PUBLISHERS *******###  
        # Create the subscriber to cmd_vel topic 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        # Create ROS publishers 
        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1) #Publisher to pose_sim topic 
        #tf_pub = tf2_ros.TransformBroadcaster()
        pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

        t = TransformStamped() 
        t2 = TransformStamped() 
        t3 = TransformStamped() 

        ############ ROBOT CONSTANTS ################  
        self.r = 0.05 #puzzlebot wheel radius [m] 
        self.L = 0.19 #puzzlebot wheel separation [m] 
        self.dt = 0.02 # Desired time to update the robot's pose [s] 
        ############ Variables ############### 
        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        self.wr = 0.0
        self.wl = 0.0
        self.x = 0.0 # 
        self.x_ant = 0.0
        self.y = 0.0  
        self.y_ant = 0.0
        self.theta = 0.0  
        self.theta_ant = 0.0

        self.odom = Odometry()

                
        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 
        while not rospy.is_shutdown(): 

            self.get_robot_vel() 
            self.update_robot_pose()
            self.get_pose_odometry(self.theta)
 
            ######## Publish the data ################# 
            self.odom_pub.publish(self.odom)
            rate.sleep() 

            t.header.stamp = rospy.Time.now() 

            t.header.frame_id = "base_link" 

            t.child_frame_id = "chassis"

            t.transform.translation.x = self.x 

            t.transform.translation.y = self.y 

            t.transform.translation.z = 0.0 


            #The transformation requires the orientation as a quaternion 

            #q = quaternion_from_euler(0, 0, theta) 

            t.transform.rotation.x = self.odom.pose.pose.orientation.x

            t.transform.rotation.y = self.odom.pose.pose.orientation.y

            t.transform.rotation.z = self.odom.pose.pose.orientation.z

            t.transform.rotation.w = self.odom.pose.pose.orientation.w

            # A transformation is broadcasted instead of published 

            #tf_pub.sendTransform(t) #broadcast the transformation 

            t2.header.stamp = rospy.Time.now() 

            t2.header.frame_id = "base_link" 

            t2.child_frame_id = "lw_link"

            t2.transform.translation.x = self.x 

            t2.transform.translation.y = self.y 

            t2.transform.translation.z = 0.0 


            #The transformation requires the orientation as a quaternion 

            #q = quaternion_from_euler(0, 0, theta) 

            t2.transform.rotation.x = self.odom.pose.pose.orientation.x

            t2.transform.rotation.y = self.odom.pose.pose.orientation.y

            t2.transform.rotation.z = self.odom.pose.pose.orientation.z

            t2.transform.rotation.w = self.odom.pose.pose.orientation.w



            t3.header.stamp = rospy.Time.now() 

            t3.header.frame_id = "base_link" 

            t3.child_frame_id = "rw_link"

            t3.transform.translation.x = self.x 

            t3.transform.translation.y = self.y 

            t3.transform.translation.z = 0.0 


            #The transformation requires the orientation as a quaternion 

            #q = quaternion_from_euler(0, 0, theta) 

            t3.transform.rotation.x = self.odom.pose.pose.orientation.x

            t3.transform.rotation.y = self.odom.pose.pose.orientation.y

            t3.transform.rotation.z = self.odom.pose.pose.orientation.z

            t3.transform.rotation.w = self.odom.pose.pose.orientation.w

            tf_message = TFMessage()
            tf_message.transforms.append(t)
            tf_message.transforms.append(t2)
            tf_message.transforms.append(t3)
        
            # Publicar el mensaje TFMessage
            pub.publish(tf_message)


     
    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data
     
    def get_robot_vel(self): 
        self.v = self.r * ((self.wr + self.wl) / 2.0)
        self.w = self.r * ((self.wr - self.wl) / self.L)

    def get_pose_odometry(self, yaw): 
        # Write the data as a ROS PoseStamped message 
        self.odom.header.frame_id = "odom"    #This can be changed in this case I'm using a frame called odom. 
        self.odom.child_frame_id = "odom2"
        self.odom.header.stamp = rospy.Time.now() 
        # Position 
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y

        # Rotation of the mobile base frame w.r.t. "odom" frame as a quaternion 
        quat = quaternion_from_euler(0, 0, yaw) 
        self.odom.pose.pose.orientation.x = quat[0] 
        self.odom.pose.pose.orientation.y = quat[1] 
        self.odom.pose.pose.orientation.z = quat[2] 
        self.odom.pose.pose.orientation.w = quat[3] 

    def update_robot_pose(self): 
        #This functions receives the robot speed v [m/s] and w [rad/s] 
        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 
        # is the orientation,     
        ############ MODIFY THIS CODE   ################ 
        self.x = self.x_ant + self.v * np.cos(self.theta) * self.dt
        self.y = self.y_ant + self.v * np.sin(self.theta) * self.dt
        self.theta = self.theta_ant + self.w * self.dt

        self.x_ant = self.x
        self.y_ant = self.y
        self.theta_ant = self.theta


 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    OdomClass()  