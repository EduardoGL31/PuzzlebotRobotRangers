#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32 

# Declare the output Messages

#q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.0)
contJoints = JointState()

class Joint_State():

    # Declare the output Messages
    def __init__(self):

        rospy.init_node("Joint_State_Publisher")
 
        # Configure the Node
        loop_rate = rospy.Rate(rospy.get_param("/rate",100))
        rospy.on_shutdown(self.stop)

        self.wl = 0.0
        self.wr = 0.0

        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)


        contJoints.header.frame_id = "chassis"
        contJoints.header.stamp = rospy.Time.now()
        contJoints.name.extend(["lw_joint", "rw_joint"])
        contJoints.position.extend([0.0, 0.0])
        contJoints.velocity.extend([0.0, 0.0])
        contJoints.effort.extend([0.0, 0.0])

        print("The Estimator is Running")
        try:
        #Run the node
            while not rospy.is_shutdown(): 
                t = rospy.Time.now().to_sec()
                contJoints.header.stamp = rospy.Time.now()
                contJoints.position[0] = self.wrap_to_Pi(self.wl)
                contJoints.position[1] = self.wrap_to_Pi(self.wr)
                
                joint_pub.publish(contJoints)

                loop_rate.sleep()
    
        except rospy.ROSInterruptException:
            pass

    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data


    #wrap to pi function
    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi

        #Stop Condition
    def stop(self):
        #Setup the stop message (can be the same as the control message)
        print("Stopping")

if __name__ =='__main__':

    Joint_State()
