#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-6 Use lidar data to walls surrounding robot
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 03, 2022
import numpy as np
# ros
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from team_x_maze.msg import Wall

####################################
## VARIABLES DECLARATION AND SETUP
####################################


# obstacle_found = True
range_max = 3.8 # 3.5
detect_range_obs = 0.5  # 0.5
detect_range_left = 0.25
fov = 8
half_window_size = fov/2
pub = rospy.Publisher('/object_pos', Wall, queue_size=1)


def modify_lidar_data(ranges):
    # modify the raw data and 
    mod_ranges = []
    for i in range(0, len(ranges) - 1):
        if ranges[i] == 0.0:
            mod_ranges.append(range_max)
        else:
            mod_ranges.append(ranges[i])
    return mod_ranges


def sliding_window_detection(ranges, half_window_size, detect_range):
    obs = []
    # print(len(ranges))
    # using sliding window to detect any possible obstacles
    for index in range(half_window_size, len(ranges) - half_window_size - 1):
        dis_x = sum(ranges[index - half_window_size: index + half_window_size]) / (2 * half_window_size + 1)
        if dis_x < detect_range:
            obs.append(index)
    # print(obs)
    real_obs = []
    #   try to denoise, if obstacles are too small, it more likes noise rather than real obstacle
    if len(obs) > 3:
        for i in range(1, len(obs)):
            if obs[i] - obs[i - 1] <= 3:  # these two points represent the same obstacle
                real_obs.append(obs[i])
    # print(real_obs)
    return real_obs


def filter_object_pose(obejct_pose, ranges):
    obs_dis = 100000.0

    if len(obejct_pose) > 2:  # or we think it's still noise
        # sum_obs = 0.0
        # center_index = len(obejct_pose) / 2
        for j in obejct_pose:
            # sum_obs = sum_obs + front[j]
            # if front[j] == 0.0:
            #     continue
            if obs_dis > ranges[j]:
                # center_index = j
                obs_dis = ranges[j]
            # if angular <0 -- right, angular >0 --left
            # angular_z = float(center_index) / (float(len(ranges))/2.0) - 1.0
        # obs_dis = sum_obs / len(real_obs)
        # center_index = len(real_obs) / 2
        # print(center_index)
    return obs_dis  # , angular_z


def lidar_callback(msg):
    # global obstacle_found
    global half_window_size
    # global obs_dis
    global pub
    obs_msg = Wall()

    if any(msg.ranges):

        # the range we care about the objects in front of robot
        front_left = msg.ranges[0:fov]
        front_right = msg.ranges[359 - fov:359]
        front = front_right + front_left
        front_mod = modify_lidar_data(front)
        real_obs = sliding_window_detection(front_mod, half_window_size, range_max)

        # Detect other walls
        left = modify_lidar_data(msg.ranges[90 - fov:  90 + fov])
        # print(np.mean(left))
        right = modify_lidar_data(msg.ranges[270 - fov:270 + fov])
        back = modify_lidar_data(msg.ranges[180 - fov: 180 + fov])

        left_wall = sliding_window_detection(left, half_window_size, range_max)
        right_wall = sliding_window_detection(right, half_window_size, range_max)
        back_wall = sliding_window_detection(back, half_window_size, range_max)

        # if len(real_obs) > 2:
        obs_msg.front = filter_object_pose(real_obs, front_mod)
        obs_msg.left = filter_object_pose(left_wall, left)
        obs_msg.right = filter_object_pose(right_wall, right)
        obs_msg.back = filter_object_pose(back_wall, back)
        # rospy.loginfo_throttle(.5,"Wall Detected, Distance = %2.2f m", obs_msg.left)

        # obs_dis, angular_z = filter_object_pose(real_obs, front_mod)
        # obs_msg.x = obs_dis # * np.cos(angular_z)
        # obs_msg.y = obs_dis * np.sin(angular_z)
        # obs_msg.z = angular_z
        # print(angular_z)
        # else:
            # obstacle_found = False
            # Data for the left side of the LiDAR for object following
            # left_side = msgs.ranges[70:105]
            # left_side_mod = modify_lidar_data(left_side)
            # left_real_obs = sliding_window_detection(left_side_mod, half_window_size, detect_range_left)
            # if len(left_real_obs) > 2:
            #     left_obs_dis, left_angular_z = filter_object_pose(left_real_obs, left_side_mod)
            #     rospy.loginfo_throttle(1.0, "Obstacle on left detected!, Distance = %2.2f m with angular = %2.2f", left_obs_dis, left_angular_z)

            #     # change these values
            #     obs_msg.x = 1000 + (left_obs_dis * np.cos(left_angular_z))
            #     obs_msg.y = 1000 + (left_obs_dis * np.sin(left_angular_z))
            #     obs_msg.z = 1000 + left_angular_z
            # else:
            # obs_msg.front = 0
            # obs_msg.left = 0
            # obs_msg.right = 0
            # obs_msg.back = 0
            # rospy.loginfo_throttle(2.0, "no obstacles found")

    pub.publish(obs_msg)


"""
create a ros node to detect obstacle position
publish the obstacle position in the world coordinate
"""


def init():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidar_node', anonymous=True)
    # position and angular subscriber

    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
