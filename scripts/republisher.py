#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy import integrate

joint_p11 = 0
joint_p12 = 0

def callback(joint_state_data):
    global joint_p11, joint_p12
    joint_p11 = joint_state_data.position[0]
    #joint_p12 = joint_state_data.position[1]

def listener():

    rospy.init_node('node_name')

    rospy.Subscriber("/hexapodo/joint_states", JointState, callback)
    joint_state_p11 = rospy.Publisher("/hexapodo/joint_states_p11", Float64, queue_size=10)
    #joint_state_p12 = rospy.Publisher("/hexapodo/joint_states_p12", Float64, queue_size=10)

    r = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        joint_state_p11.publish(joint_p11)
        #joint_state_p12.publish(joint_p12)

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
