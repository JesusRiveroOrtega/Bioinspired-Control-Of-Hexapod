#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from std_msgs.msg import Float64
import math
import numpy as np
from matplotlib import pyplot as plt

def talker():
	pub=[]
	rospy.init_node('hexapod_talker', anonymous=True)
	rate = rospy.Rate(10000) # 10hz
	for k in range(1,7):
		for l in range(1,4):
			pub.append(rospy.Publisher('/hexapodo/jp'+str(k)+str(l)+'_position_controller/command', Float64, queue_size=10))
	for t in range(1, 200):
		z=0	
		for j in range(1,7):
			for i in range(1,4):
				pub[z].publish(0)
				rate.sleep()
				z+= 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
