#!/usr/bin/env python
from __future__ import division
import roslib
import sys, time
import rospy
import numpy as np
from std_msgs.msg import String
from hexapodo.msg import Num




def main(args):

    sum_pub = rospy.Publisher("/imu", Num, queue_size = 1)
    rospy.init_node('imu', anonymous = True)
    rate = rospy.Rate(0.1) # 10hz

    k = 0

    ang = 0
    while not rospy.is_shutdown():




        if k >= 1:
            sum_pub.publish(np.array([0, 0]))
        else:
            ang += np.random.randint(-45, 45)
            sum_pub.publish(np.array([1, 40]))


        k += 1
        print(k)
        rate.sleep()


if __name__ == '__main__':
	main(sys.argv)
