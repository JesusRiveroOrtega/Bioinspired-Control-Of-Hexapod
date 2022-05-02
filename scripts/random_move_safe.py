#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import sys, select, termios, tty
import numpy as np


from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hexapodo.msg import Num
from std_msgs.msg import String

x = 0
y = 0
z = 0
th = 0
reached = False
yaw = 0
mode = ""
angle = 0
distance = 0
def ActionSelection(as_data):
    global mode
    mode = as_data.data

def VectorSum(sum):
    global angle, distance
    vec = sum.num
    angle = vec[2]
    distance = vec[3]

def callback(imu_data):
    global yaw

    orientation = imu_data.orientation
    orientation_l = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_l)
    yaw = np.rad2deg(yaw)
    yaw = (yaw - 360 * np.fix(yaw / 360))*(yaw > 0) + (yaw + 360) * (yaw < 0)

def current_distance(data):
    global current_distance
    current_distance = data.num
    current_distance = current_distance[0]

def listener():
    global x, y, z, th, reached, mode, current_distance
    rospy.init_node('random_mover')
    rospy.Subscriber("/imu/data", Imu, callback)
    rospy.Subscriber("/ActionSelection", String, ActionSelection)
    rospy.Subscriber("/VectorSum", Num, VectorSum)
    Twister = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    int_en = rospy.Publisher('/int_enable', Num, queue_size = 1)
    escape_en = rospy.Publisher('/escape_en', Num, queue_size = 1)
    direct = rospy.Publisher('/hexapod_direct', Num, queue_size = 1)
    rospy.Subscriber("/distance", Num, current_distance)

    r = rospy.Rate(100) # 10hz
    a = -1
    b = 1
    speed = 0.25
    t = 1
    twist = Twist()
    error_val = [0, 0]
    d_error_val = 0
    turn = 0
    x = 0
    k = 0
    enable = 1
    set_point = 180
    en = 0
    f = 0
    f2 = 0
    start = 0
    s = 0
    p = 1
    w = 0
    mode_engage = 0
    random_mov = 0
    while not rospy.is_shutdown():
        if start > 1000:
            if mode == "Explorar":

                escape_en.publish(np.array([0]))
                if reached == False:
                    f2 = 0
                    if f < 500:
                        pass

                    else:
                        if en == 0:
                            int_en.publish(np.array([enable]))
                            en = 1
                        if set_point > 360:
                            y = np.fix(set_point / 360)
                            set_point -= y*360

                        if set_point < 0:
                            set_point += 360

                        error_val[1] = set_point - yaw
                        th = 1
                        turn = 0.2*error_val[1]/np.abs(error_val[1])

                        print('----------')
                        print(set_point)
                        print(yaw)
                        print(error_val[1])
                        print(turn)
                        print('----------')
                        t = 0

                        if error_val[1]> -0.15 and error_val[1] < 0.15:
                            reached = True
                            turn = 0
                            k = 0
                            f = 0
                            random_mov = np.random.randint(1000, 1500)
                            #random_mov = 2000


                    f += 1


                if reached == True:
                    f = 0

                    en = 0
                    if f2 < 500:
                        pass
                    else:
                        if k < random_mov:
                            x = 1
                        else:

                            x = 0
                            set_point += np.random.randint(-45, 45)

                            reached = False

                    f2 += 1
                    k += 1




            else:

                if p == 1:
                    reached = False
                    mode_engage = 1
                    p = 0
                    x = 0


                if reached == False:
                    f2 = 0
                    if f < 50:
                        pass

                    else:
                        set_point = 180 + angle

                        if set_point > 360:
                            y = np.fix(set_point / 360)
                            set_point -= y*360

                        if set_point < 0:
                            set_point += 360

                        error_val[1] = set_point - yaw
                        th = 1
                        turn = 0.2*error_val[1]/np.abs(error_val[1])
                        x = 0
                        print('----------')
                        print(set_point)
                        print(yaw)
                        print(error_val[1])
                        print(turn)
                        print('----------')
                        t = 0

                        if error_val[1]> -0.15 and error_val[1] < 0.15:
                            reached = True
                            turn = 0
                            k = 0
                            f = 0

                    f += 1


                if reached == True:
                    f = 0
                    escape_en.publish(np.array([1]))
                    en = 0
                    if f2 < 1000:
                        pass
                    else:
                        if w == 100:
                            int_en.publish(np.array([enable]))

                            w = 0
                        w += 1

                        if current_distance < random_mov:
                            x = 1
                        else:

                            x = 0
                            set_point += 0



                    f2 += 1
                    k += 1

        start += 1

        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn


        Twister.publish(twist)


        error_val[0] = error_val[1]

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
