#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division
#import roslib; roslib.load_manifest('teleop_twist_keyboard')
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
from std_msgs.msg import Int64

yaw = 0
mode = ""
angle = 0
distance = 0
setpoint = 0
left = 0
right = 0
rotation_direction = 2
insta_distance = 0
return_angle = 0
return_distance = 0
input_angle = 0
distance_blue = 0
distance_yummy = 0

def Rotation_direction(data):
    global rotation_direction
    rotation_direction = data.data

def ActionSelection(as_data):
    global mode
    mode = as_data.data

def callback(imu_data):
    global yaw

    orientation = imu_data.orientation
    orientation_l = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_l)
    yaw = np.rad2deg(yaw)
    yaw = (yaw - 360 * np.fix(yaw / 360))*(yaw > 0) + (yaw + 360) * (yaw < 0)

def VectorSum(data):
    global return_angle, return_distance
    return_angle = data.num[2] +180
    return_distance = data.num[3]

def Insta_distance(data):
    global insta_distance
    insta_distance = data.data

def Setpoint(data):
    global input_angle
    ing = data.num

    input_angle = ing[0]

def Distance_Blue(data):
    global distance_blue
    distance_blue = data.data

def listener():
    global setpoint, return_angle, return_distance
    rospy.init_node('attack')
    rospy.Subscriber("/imu", Imu, callback)
    rospy.Subscriber("/rotation_direction", Int64, Rotation_direction)
    rospy.Subscriber("/ActionSelection", String, ActionSelection)
    rospy.Subscriber("/VectorSum", Num, VectorSum)
    rospy.Subscriber("/insta_distance", Float64, Insta_distance)
    rospy.Subscriber("/distance_blue", Float64, Distance_Blue)
    traj_integration_input = rospy.Publisher('/traj_integration_input', Num, queue_size=1)
    insta_distance_reset = rospy.Publisher('/insta_distance_reset', Int64, queue_size=1)
    orientation_msg = rospy.Publisher('/angle_current', Float64, queue_size=1)
    angle_setpoint = rospy.Publisher('/angle_setpoint', Float64, queue_size=1)
    hexapod_direct = rospy.Publisher('/hexapod_direct', Num, queue_size=1)
    rospy.Subscriber("/setpoint", Num, Setpoint)
    r = rospy.Rate(100) # 10hz
    wait_walk = 0
    time_orientation = 0
    time_walking = 0
    oriented = False
    wait_begin = 0
    p = 1
    set_point = 0

    while not rospy.is_shutdown():

        if mode == "attack":
            if p == 1:
                reached = False
                p = 0
                insta_distance_reset.publish(1)
                hexapod_direct.publish(np.array([1, 1]))
                set_point = yaw - input_angle
                distance_yummy = distance_blue

            if wait_begin < 40:
                wait_begin += 1
            else:
                orientation_msg.publish(yaw)
                angle_setpoint.publish(set_point)

                if oriented == False:
                    if rotation_direction == 1:
                        hexapod_direct.publish(np.array([1, 0]))

                    elif rotation_direction == -1:
                        hexapod_direct.publish(np.array([0, 1]))

                    elif rotation_direction == 0:
                        hexapod_direct.publish(np.array([1, 1]))
                        oriented = True
                    else:
                        pass

                if oriented == True:
                    if wait_walk < 40:
                        wait_walk += 1
                    else:
                        print([insta_distance, distance_yummy])
                        if insta_distance < 0.95*distance_yummy:

                            hexapod_direct.publish(np.array([0, 0]))
                            # oriented = False
                        else:

                            hexapod_direct.publish(np.array([1, 1]))



        else:
            p = 1

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
