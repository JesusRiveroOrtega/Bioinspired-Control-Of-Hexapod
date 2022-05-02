#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from std_msgs.msg import Float64
import math
import numpy as np
from matplotlib import pyplot as plt


pub=[]
data=0.0
h = 1
stime = 1000000
time = np.linspace(0, stime, int(stime/h))


def RK4(stime, h):
	n = int(stime/h)

        z1 = np.zeros((18,n))

	cont1=0
	cont2=0
	cont3=0
	cont4=10
        S1=1
        S2=1
        S3=-1
        S4=1
	rospy.init_node('hexapod_talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
        H=0

	for k in range(1,7):
		for l in range(1,4):
			pub.append(rospy.Publisher('/hexapodo/jp'+str(k)+str(l)+'_position_controller/command', Float64, queue_size=10))

	for t in range(1, n):
		

                cont1=cont1 +1
                cont2=cont2 +1
                cont3=cont3 +1
                cont4=cont4 +1

                if cont1==50 and S1==1:
                    S1=-1
                    cont1=0
                if cont1==50 and S1==-1:
                    S1=1
                    cont1=0
                if cont2==40 and S2==1:
                    S2=-1
                    cont2=0
                if cont2==60 and S2==-1:
                    S2=1
                    cont2=0

		if cont3==50 and S3==1:
		    S3=-1
		    cont3=0
		if cont3==50 and S3==-1:
		    S3=1
		    cont3=0
		if cont4==60 and S4==1:
		    S4=-1
		    cont4=0
		if cont4==40 and S4==-1:
		    S4=1
		    cont4=0

		z=0
		for i in range(0,6):
			for j in range(0,3):

                                z1[0,t]=S1*0.4
                                z1[1,t]=-S2*0.5+0.5
                                z1[2,t]=0

                                z1[3,t]=S3*0.4
                                z1[4,t]=S4*-0.5+0.5
                                z1[5,t]=0

                                z1[6,t]=S1*0.4
                                z1[7,t]=-S2*0.5+0.5
                                z1[8,t]=0

                                z1[9,t]=-S3*0.4
                                z1[10,t]=S4*-0.5+0.5
                                z1[11,t]=0

                                z1[12,t]=-S1*0.4
                                z1[13,t]=-S2*0.5+0.5
                                z1[14,t]=0

                                z1[15,t]=-S3*0.4
                                z1[16,t]=S4*-0.5+0.5
                                z1[17,t]=0

                                pub[z].publish(z1[z,t])
				print(z1[z,t])
				print(z)



				z+=1
	   	rate.sleep()
				



def talker():
	t=0
	jp=([11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63])
	RK4(stime, h)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

