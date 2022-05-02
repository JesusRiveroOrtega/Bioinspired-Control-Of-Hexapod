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
from std_msgs.msg import Float64

x = 0
y = 0
z = 0
th = 0
reached = False
yaw = 0
mode = ""
angle = 0
distance = 0
oriented = 0
left = 0
right = 0


def Left(data):
    global left
    left = data.data

def Right(data):
    global right
    right = data.data

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

def Setpoint(data):
    global input_angle
    ing = data.num

    input_angle = ing[0]

def listener():
    global x, y, z, th, reached, mode, current_distance, oriented
    rospy.init_node('random_mover')
    rospy.Subscriber("/imu", Imu, callback)
    rospy.Subscriber("/left", Float64, Left)
    rospy.Subscriber("/right", Float64, Right)
    rospy.Subscriber("/ActionSelection", String, ActionSelection)
    rospy.Subscriber("/VectorSum", Num, VectorSum)
    Twister = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    int_en = rospy.Publisher('/int_enable', Num, queue_size = 1)
    escape_en = rospy.Publisher('/escape_en', Num, queue_size = 1)
    direct = rospy.Publisher('/hexapod_direct', Num, queue_size = 1)
    rospy.Subscriber("/distance", Num, current_distance)
    hex_dir = rospy.Publisher('/hexapod_direct', Num, queue_size = 1)
    sp= rospy.Publisher('/setpoint_q', Float64, queue_size = 1)
    rospy.Subscriber("/setpoint", Num, Setpoint)
    acc_raw = rospy.Publisher('/distance', Num, queue_size=1)
    orientation_msg = rospy.Publisher('/orientation', Num, queue_size=1)


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
    set_point = 0
    en = 0
    f = 0
    f2 = 0
    start = 0
    s = 0
    p = 1
    w = 0
    eng = 0
    mode_engage = 0
    hh = 0
    random_mov = 0
    there = False
    while not rospy.is_shutdown():

        if mode == "Explorar":
            sp.publish(set_point)
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

                    # error_val[1] = set_point - yaw
                    # th = 1
                    # turn = 0.2*error_val[1]/np.abs(error_val[1])
                    # direct.publish(np.array([0, 1]))

                    if left > 1.02*right:
                        hex_dir.publish(np.array([1, 0]))
                        print("left -- - ")
                    elif right > 1.02*left:
                        hex_dir.publish(np.array([0, 1]))
                        print("right -- - ")
                    else:
                        hex_dir.publish(np.array([1, 1]))
                        print("stop -- - ")
                        random_mov = 4000
                        reached = True
                        # direct.publish(np.array([1, 1]))
                        k = 0
                        f = 0

                    # print('----------')
                    # print(set_point)
                    # print(yaw)
                    # print('-  -  -')
                    # print([left, right])
                    # print('----------')
                    t = 0

                    # if error_val[1]> -1.5 and error_val[1] < 1.5:
                    #     reached = True
                    #     turn = 0
                    #     direct.publish(np.array([1, 1]))
                    #     k = 0
                    #     f = 0
                    #     #random_mov = np.random.randint(3000, 3500)
                    #     random_mov = 4000


                f += 1


            if reached == True:
                f = 0

                en = 0
                if f2 < 500:
                    pass
                else:
                    if k < random_mov:
                        x = 1
                        direct.publish(np.array([0, 0]))
                    else:

                        x = 0
                        direct.publish(np.array([1, 1]))
                        acc_raw.publish(np.array([1.6]))
                        orientation_msg.publish(np.array([yaw]))
                        set_point += np.random.randint(-45, 45)
                        # set_point += 0
                        reached = False

                f2 += 1
                k += 1




        # if mode == "Huir" and eng==0 :
        #     print(mode)
        #     if p == 1:
        #         reached = False
        #         mode_engage = 1
        #         p = 0
        #         x = 0
        #         direct.publish(np.array([1, 1]))
        #
        #
        #     if reached == False:
        #         f2 = 0
        #         if f < 50:
        #             pass
        #
        #         else:
        #             set_point = 180 + angle
        #             sp.publish(set_point)
        #
        #             if set_point > 360:
        #                 y = np.fix(set_point / 360)
        #                 set_point -= y*360
        #
        #             if set_point < 0:
        #                 set_point += 360
        #
        #             if left > 1.02*right:
        #                 hex_dir.publish(np.array([1, 0]))
        #                 print("left -- - ")
        #             elif right > 1.02*left:
        #                 hex_dir.publish(np.array([0, 1]))
        #                 print("right -- - ")
        #             else:
        #                 hex_dir.publish(np.array([1, 1]))
        #                 print("stop -- - ")
        #                 random_mov = 4000
        #                 reached = True
        #                 k = 0
        #                 f = 0
        #
        #             # error_val[1] = set_point - yaw
        #             # hex_dir.publish(np.array([0, 1]))
        #             # th = 1
        #             # turn = 0.2*error_val[1]/np.abs(error_val[1])
        #             # x = 0
        #             # print('----------')
        #             # print(set_point)
        #             # print(yaw)
        #             # print(error_val[1])
        #             # print(turn)
        #             # print('----------')
        #             t = 0
        #
        #             # if error_val[1]> -0.15 and error_val[1] < 0.15:
        #             #     reached = True
        #             #     turn = 0
        #             #     k = 0
        #             #     f = 0
        #
        #         f += 1
        #
        #
        #     if reached == True:
        #         f = 0
        #         escape_en.publish(np.array([1]))
        #         en = 0
        #         if f2 < 1000:
        #             pass
        #         else:
        #             if w == 100:
        #                 int_en.publish(np.array([enable]))
        #
        #                 w = 0
        #             w += 1
        #
        #             if current_distance < random_mov:
        #                 x = 1
        #                 hex_dir.publish(np.array([0, 0]))
        #             else:
        #                 hex_dir.publish(np.array([1, 1]))
        #                 x = 0
        #                 set_point += 0
        #
        #
        #
        #         f2 += 1
        #         k += 1
        #
        # if mode == "Atacar" :
        #
        #     # print(mode)
        #     if p == 1:
        #         reached = False
        #         eng = 1
        #         p = 0
        #         x = 0
        #         f = 0
        #         direct.publish(np.array([1, 1]))
        #         set_point = yaw - input_angle
        #
        #     if reached == False:
        #         f2 = 0
        #         if f < 50:
        #             pass
        #
        #         else:
        #
        #             sp.publish(set_point)
        #             print([set_point, yaw])
        #
        #
        #             if left > 1.02*right:
        #                 hex_dir.publish(np.array([1, 0]))
        #                 print("left -- - ")
        #             elif right > 1.02*left:
        #                 hex_dir.publish(np.array([0, 1]))
        #                 print("right -- - ")
        #             else:
        #                 hex_dir.publish(np.array([1, 1]))
        #                 print("stop -- - ")
        #                 random_mov = 4000
        #                 if np.abs(yaw - set_point) < 2.5:
        #                     reached = True
        #                 k = 0
        #                 f = 0
        #
        #
        #         f += 1
        #
        #     if reached == True:
        #         f = 0
        #         hex_dir.publish(np.array([0, 0]))
        #
        #         set_point += 0
        #
        #
        #
        #         f2 += 1
        #         k += 1





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
