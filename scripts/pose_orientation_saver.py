#!/usr/bin/env python3


from __future__ import division
from __future__ import print_function
import rospy
from std_msgs.msg import Float64, Int64, String
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import LinkStates
import math
import numpy as np
from matplotlib import pyplot as plt
from hexapodo.msg import Num
from tf.transformations import euler_from_quaternion

link_names = String()
link_poses = Pose()
link_orien = Twist()

def get_pose_from_topic(data):
    global link_names, link_poses, link_orien
    
    link_names = data.name[1]
    link_poses = data.pose[1].position
    link_orien = euler_from_quaternion([data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])
    



def pose_logger():
    rospy.init_node('pose_logger', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose_from_topic)
    rospy.Publisher("/pruebitas", String, queue_size = 1)
    rospy.Rate(0.01)
    while not rospy.is_shutdown():
        print(link_names)
        print(link_poses)
        print(link_orien)
        




if __name__ == '__main__':
	try:
		pose_logger()
	except rospy.ROSInterruptException:
		pass
