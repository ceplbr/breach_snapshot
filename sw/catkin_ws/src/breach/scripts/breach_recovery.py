#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

# global variables

global speed, status, goal, speed_recovery, cmd_speed
global speed_list, speed_avg, moveBase_status
global moveBase_status, time_last_recovery
global srf_01_dist, srf_02_dist, srf_03_dist, srf_04_dist, srf_05_dist, srf_06_dist, srf_07_dist, srf_08_dist, srf_09_dist, srf_10_dist
global laser_data

srf_01_dist = srf_02_dist = srf_03_dist = srf_04_dist = srf_05_dist = srf_06_dist = srf_07_dist = srf_08_dist = srf_09_dist = srf_10_dist = laser_data = None

speed_list_size = 100

result = 3
moveBase_status = 0

# avg speed
speed_avg = 0.1
speed_list = []
for i in range(speed_list_size):
    speed_list.append(0.1)

speed = Twist()
goal = PoseStamped()
cmd_speed = Twist()
speed_recovery = Twist()
speed_recovery.linear.x = -0.1

# ===== CALLBACKS ========================

def callback_speed_left(data):
    global speed_avg, speed_list
    speed_list.append(abs(data.data))
    del speed_list[0]
    speed_avg = np.mean(speed_list)

def callback_speed_right(data):
    global speed_avg, speed_list
    speed_list.append(abs(data.data))
    del speed_list[0]
    speed_avg = np.mean(speed_list)

def callback_status(data):
    global moveBase_status, time_last_recovery
    if len(data.status_list):
        moveBase_status = data.status_list[0].status
    else:
        moveBase_status = 0
        time_last_recovery = rospy.get_time()

#~ def callback_result(data):
    #~ global result
    #~ result = data.status.status

#~ def callback_goal(data):
    #~ global goal
    #~ goal = data

#~ def callback_cmd_speed(data):
    #~ global cmd_speed
    #~ cmd_speed = data

def callback_srf01(data):
    global srf_01_dist
    srf_01_dist = data.range

def callback_srf02(data):
    global srf_02_dist
    srf_02_dist = data.range

def callback_srf03(data):
    global srf_03_dist
    srf_03_dist = data.range

def callback_srf04(data):
    global srf_04_dist
    srf_04_dist = data.range

def callback_srf05(data):
    global srf_05_dist
    srf_05_dist = data.range

def callback_srf06(data):
    global srf_06_dist
    srf_06_dist = data.range

def callback_srf07(data):
    global srf_07_dist
    srf_07_dist = data.range

def callback_srf08(data):
    global srf_08_dist
    srf_08_dist = data.range

def callback_srf09(data):
    global srf_09_dist
    srf_09_dist = data.range

def callback_srf10(data):
    global srf_10_dist
    srf_10_dist = data.range

def callback_laserscan(data):
    global laser_data
    laser_data = data

def my_recovery():
    global speed_list, speed_avg, moveBase_status, time_last_recovery
    global srf_01_dist, srf_02_dist, srf_03_dist, srf_04_dist, srf_05_dist, srf_06_dist, srf_07_dist, srf_08_dist, srf_09_dist, srf_10_dist
    global laser_data

    initialized = False;
    cmd_recovery_values = Twist()

    rospy.init_node('my_recovery', anonymous=True) # jak se jmenuje tenhle uzel - nemusi to byt nazev skriptu

    rospy.Subscriber("/breach/motor/left/speed", Float32, callback_speed_left)
    rospy.Subscriber("/breach/motor/right/speed", Float32, callback_speed_right)

    rospy.Subscriber("/move_base/status", GoalStatusArray, callback_status) # /move_base/status; GoalStatusArray - K PREPSANI
    #rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback_result) # /move_base/result; MoveBaseActionResult - K PREPSANI
    #rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_goal) # /move_base_simple/goal; PoseStamped
    #rospy.Subscriber("/cmd_vel", Twist, callback_cmd_speed)

    rospy.Subscriber("/ultrasounds/srf01", Range, callback_srf01)
    rospy.Subscriber("/ultrasounds/srf02", Range, callback_srf02)
    rospy.Subscriber("/ultrasounds/srf03", Range, callback_srf03)
    rospy.Subscriber("/ultrasounds/srf04", Range, callback_srf04)
    rospy.Subscriber("/ultrasounds/srf05", Range, callback_srf05)
    rospy.Subscriber("/ultrasounds/srf06", Range, callback_srf06)
    rospy.Subscriber("/ultrasounds/srf07", Range, callback_srf07)
    rospy.Subscriber("/ultrasounds/srf08", Range, callback_srf08)
    rospy.Subscriber("/ultrasounds/srf09", Range, callback_srf09)
    rospy.Subscriber("/ultrasounds/srf10", Range, callback_srf10)
    rospy.Subscriber("/scan", LaserScan, callback_laserscan)

    pubRecoveryInfo = rospy.Publisher('/breach/error/recovery', Bool, queue_size = 10)
    pubRecoveryCmd = rospy.Publisher('/cmd_recovery', Twist, queue_size = 10)

    list_speed = list()
    list_cmd_speed = list()

    time_last_recovery = rospy.get_time()
    start_time = rospy.get_time()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        speed_tol = 0.001

        if (speed_avg < speed_tol and moveBase_status == 1 and rospy.get_time() - time_last_recovery > 10.0) or initialized:

            if initialized and rospy.get_time() - start_time > 8.0:
                print("Recovery done!")
                initialized = False
                time_last_recovery = rospy.get_time()
                r.sleep()
                continue

            if not initialized and rospy.get_time() - start_time > 8.0:
                initialized = True
                start_time = rospy.get_time()
                print("Recovery mode!")

            sonars_array =   [srf_01_dist if srf_01_dist else 6.0,
                              srf_02_dist if srf_02_dist else 6.0,
                              srf_03_dist if srf_03_dist else 6.0,
                              srf_04_dist if srf_04_dist else 6.0,
                              srf_05_dist if srf_05_dist else 6.0,
                              srf_06_dist if srf_06_dist else 6.0,
                              srf_07_dist if srf_07_dist else 6.0,
                              srf_08_dist if srf_08_dist else 6.0,
                              srf_09_dist if srf_09_dist else 6.0,
                              srf_10_dist if srf_10_dist else 6.0 ]

            if laser_data:
                sonars_min = min(sonars_array)
                laser_ranges = []
                for sample_range in laser_data.ranges:
                    if sample_range > laser_data.range_min and sample_range < laser_data.range_max:
                        laser_ranges.append(sample_range)
                lidar_min = min(laser_ranges)

                if lidar_min < sonars_min:
                    obstacle_pos = laser_data.ranges.index(lidar_min)
                    obstacle_angle = laser_data.angle_min + obstacle_pos*laser_data.angle_increment

                    if lidar_min < 0.4:
                        if abs(obstacle_angle) < -0.75: # obstacle on right
                            cmd_recovery_values.linear.x = 0.0
                            cmd_recovery_values.angular.z = -0.2
                        elif abs(obstacle_angle) > 0.75: # obstacle on left
                            cmd_recovery_values.linear.x = 0.0
                            cmd_recovery_values.angular.z = 0.2
                        else:
                            cmd_recovery_values.linear.x = -0.1
                            cmd_recovery_values.angular.z = 0.0
                    else:
                        cmd_recovery_values.linear.x = -0.1
                        cmd_recovery_values.angular.z = 0.0
                else:
                    sonar_number = sonars_array.index(min(sonars_array)) + 1
                    if sonar_number == 5 and srf_05_dist < 0.2:
                        cmd_recovery_values.linear.x = -0.1
                        cmd_recovery_values.angular.z = 0.0
                    elif sonar_number == 1 and srf_01_dist < 0.2:
                        cmd_recovery_values.linear.x = 0.05
                        cmd_recovery_values.angular.z = 0.1
                    elif sonar_number == 9 and srf_09_dist < 0.2:
                        cmd_recovery_values.linear.x = 0.05
                        cmd_recovery_values.angular.z = -0.1
                    elif (sonar_number == 2 and srf_02_dist < 0.2) or (sonar_number == 3 and srf_03_dist < 0.3):
                        cmd_recovery_values.linear.x = 0.00
                        cmd_recovery_values.angular.z = -0.2
                    elif (sonar_number == 8 and srf_08_dist < 0.2) or (sonar_number == 7 and srf_07_dist < 0.3):
                        cmd_recovery_values.linear.x = 0.00
                        cmd_recovery_values.angular.z = 0.2
                    elif sonar_number == 4 and srf_04_dist < 0.2:
                        cmd_recovery_values.linear.x = -0.05
                        cmd_recovery_values.angular.z = 0.1
                    elif sonar_number == 6 and srf_06_dist < 0.2:
                        cmd_recovery_values.linear.x = -0.05
                        cmd_recovery_values.angular.z = -0.1
                    else:
                        cmd_recovery_values.linear.x = 0.1
                        cmd_recovery_values.angular.z = 0.0

                pubRecoveryInfo.publish(True)
                pubRecoveryCmd.publish(cmd_recovery_values)

        r.sleep()

if __name__ == '__main__':
    try:
        my_recovery()
    except rospy.ROSInterruptException:
        pass


