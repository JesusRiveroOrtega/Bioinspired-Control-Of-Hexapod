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
import csv
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

link_names = String()
link_poses = []
link_orien = ()
pose_saver_enabler = 0

def Rotation_direction(data):
    global rotation_direction
    rotation_direction = data.data

def ActionSelection(as_data):
    global mode
    mode = as_data.data

def get_pose_from_topic(data):
    global link_names, link_poses, link_orien, pose_saver_enabler
    
    link_names = data.name[1]
    link_poses = [data.pose[1].position.x, data.pose[1].position.y]
    link_orien = euler_from_quaternion([data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])

    line = [str(link_poses[0]), str(link_poses[1]), str(link_orien[0]), str(link_orien[1]), str(link_orien[2])]
    if pose_saver_enabler == 1:
        with open("/home/jesusrivero/ttbot/src/hexapodo/data/CSV files/hexapod_pose_5.csv", "a") as file:
            file.write("--. \n")
            for element in line:
                file.write(element)
                file.write(",")
            file.write("\n")
        pose_saver_enabler = 0
    elif pose_saver_enabler == 2:
        with open("/home/jesusrivero/ttbot/src/hexapodo/data/CSV files/hexapod_pose_5.csv", "a") as file:
            file.write("__. \n")
            for element in line:
                file.write(element)
                file.write(",")
            file.write("\n")
        pose_saver_enabler = 0

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

def listener():
    global setpoint, return_angle, return_distance, pose_saver_enabler
    rospy.init_node('escape')
    rospy.Subscriber("/imu", Imu, callback)
    rospy.Subscriber("/rotation_direction", Int64, Rotation_direction)
    rospy.Subscriber("/ActionSelection", String, ActionSelection)
    rospy.Subscriber("/VectorSum", Num, VectorSum)
    rospy.Subscriber("/insta_distance", Float64, Insta_distance)
    traj_integration_input = rospy.Publisher('/traj_integration_input', Num, queue_size=1)
    insta_distance_reset = rospy.Publisher('/insta_distance_reset', Int64, queue_size=1)
    orientation_msg = rospy.Publisher('/angle_current', Float64, queue_size=1)
    angle_setpoint = rospy.Publisher('/angle_setpoint', Float64, queue_size=1)
    hexapod_direct = rospy.Publisher('/hexapod_direct', Num, queue_size=1)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose_from_topic)
    r = rospy.Rate(100) # 10hz
    wait_walk = 0
    time_orientation = 0
    time_walking = 0
    oriented = False
    wait_begin = 0
    p = 1
    pse_enabler = 1

    


    while not rospy.is_shutdown():

        if mode == "escape":
            
            # if p == 1:
            #     reached = False
            #     p = 0
            #     traj_integration_input.publish(np.array([insta_distance, yaw]))
            #     insta_distance_reset.publish(1)

            if wait_begin < 400:
                wait_begin += 1
                if wait_begin == 399:
                    pose_saver_enabler = 1
                    print("enabler 1")
            else:
                orientation_msg.publish(yaw)
                angle_setpoint.publish(return_angle)
                print(oriented)
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
                    if wait_walk < 100:
                        wait_walk += 1
                        
                    else:
                        if insta_distance < return_distance:
                            hexapod_direct.publish(np.array([0, 0]))
                            # oriented = False
                            
                            

                        else:
                            # insta_distance_reset.publish(1)
                            hexapod_direct.publish(np.array([1, 1]))
                            wait_walk = 0
                            wait_being = 0
                            if pse_enabler == 1:
                                pose_saver_enabler = 2
                                pse_enabler = 0
                            print("enabler 2")

                        # if rotation_direction == 2 or rotation_direction == -2:
                        #     oriented = False
                        #     angle_setpoint.publish(return_angle)
        else:
            p = 1

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
