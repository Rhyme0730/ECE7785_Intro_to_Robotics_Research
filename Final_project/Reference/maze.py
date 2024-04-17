#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Final Lab Demo
Integrate Multi-modal sensing and Navigation into robot 
to complete a maze by following signs

Author: Yinuo Wang, Praneeth Eddu
Date: 04/14/2022
'''

"""
TODO(✔️ ❌) :  
1. Integrate KNN model into ROS to classify signs using Raspi camera ❌
2. Integrate KNN model into ROS to classify signs using Sim camera ✔️ 
3. Use LiDAR to determine distance to walls ✔️
4. Leverage distance and signage data to publish goals using move_base simple goal ✔
5. Integrate camera in simulation environment ✔️
6. Check for target sign of completion ✔️
7. Develop a failsafe in-case robot gets lost ❌
8. Define the actions when detecting "not enter" and "stop" sign ✔
"""

# ROS 
import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import Point
from team_x_maze.msg import Wall
from actionlib_msgs.msg import GoalStatusArray

# Python
import sys
import roslib
import tf
# roslib.load_manifest('learning_tf')
import numpy as np
import knn
import cv2
from cv_bridge import CvBridge
import math
import time

###########################################
#            Global variables            ##
###########################################
sim = False
bridge = CvBridge()  # Bridge converts the image from ros to openCV
DEBUG = False
# DEBUG = True

# sign
sign_dict = {
    -1: "NA",
    0: "Wall",
    1: "Left",
    2: "Right",
    3: "DoNotEnter",
    4: "Stop",
    5: "Goal"
}
sign = -1
dist_to_wall = [0, 0, 0, 0]

# pos in map frame
cur_pos = [0, 0, 0]
cur_ang = 0
# heading define
#  ----------                 ^  front
#  |        |                 |
#  |   M    |      left   <-     ->   right
#  |   A    |                 |
#  |   P    |                 v  back
#  |        |
#  ----------
left_heading = [0, 0, 0.707, 0.707]
right_heading = [0, 0, -0.707, 0.707]
forward_heading = [0, 0, 0, 1]
backward_heading = [0, 0, 1, 0]

heading = [left_heading, forward_heading, right_heading, backward_heading]
cur_heading = 1  # fwd
scan_angle_inc = 0.117
lost = False
lost_count = 0
max_pred_dist = 0.5334 # (in m) max distance where the camera can predict signs accurately
new_heading = [9999, 9999, 9999, 9999]
wall_count = 0

# goal
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
goal = PoseStamped()
goal_offset = 0.3
initial = True
 
 # misc
start_time = time.time() 
"""
Check the robot arrival status
"""


def check_arrive():
    global cur_pos
    global cur_ang
    global goal
    
    ang = qua2rpy(heading[cur_heading])
    # check if arrive, please keep following parameters are the same as them in dwa_local_planner_params.yaml
    if abs(goal.pose.position.x - cur_pos[0]) < 0.10 and abs(goal.pose.position.y - cur_pos[1]) < 0.10 and (abs(
            ang - cur_ang) < 10 or abs(ang - cur_ang) > 350):  # degree (0-360)
            return True
    # rospy.loginfo("%f, %f, %f",abs(goal.pose.position.x - cur_pos[0]),abs(goal.pose.position.y - cur_pos[1]),abs(ang - cur_ang))
    return False

def get_transform():
    global cur_pos
    global cur_ang
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            cur_pos[0] = trans[0]
            cur_pos[1] = trans[1]
            cur_pos[2] = trans[2]
            cur_ang = qua2rpy(rot)
            # rospy.loginfo("%f %f %f", cur_pos[0], cur_pos[1], cur_pos[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
"""
set next goal in map frame
"""


def set_goal():
    global goal
    global heading
    global cur_heading
    global cur_pos
    global dist_to_wall
    global initial
    global lost
    global new_heading
    global lost_count
    global start_time
    global wall_count

    if check_arrive() or initial:
        initial = False
        goal.header.frame_id = "map"
        rospy.loginfo("%s Sign Detected", sign_dict[sign])
        rospy.loginfo("(%f %f) %f", cur_pos[0], cur_pos[1], dist_to_wall[2])
        dist = dist_to_wall
	
	    # check if the robot has been close to the wall
        if dist_to_wall[0] >= 0.6 and not initial: #still having big distance from the wall 
            wall_count = 0
            lost = False
            dis_tmp = 0.45
            rospy.loginfo("Dis to wall: %f, current pos (%f, %f)", dist_to_wall[0], cur_pos[0], cur_pos[1])
            if cur_heading == 0: # facing left now 
                goal.pose.position.y = cur_pos[1] + (dist_to_wall[0]-dis_tmp)
                goal.pose.position.x = cur_pos[0]
            elif cur_heading == 1: #facing forwards
                goal.pose.position.x = cur_pos[0] + (dist_to_wall[0]-dis_tmp)
                goal.pose.position.y = cur_pos[1]
            elif cur_heading == 2: #facing right
                goal.pose.position.y = cur_pos[1] - (dist_to_wall[0]-dis_tmp)
                goal.pose.position.x = cur_pos[0]
            else: # facing backwards
                goal.pose.position.x = cur_pos[0] - (dist_to_wall[0]-dis_tmp)
                goal.pose.position.y = cur_pos[1]
            goal.pose.orientation.x = heading[cur_heading][0]
            goal.pose.orientation.y = heading[cur_heading][1]
            goal.pose.orientation.z = heading[cur_heading][2]
            goal.pose.orientation.w = heading[cur_heading][3]
            rospy.loginfo("New Goal: (%f, %f), (%f,%f,%f,%f)", goal.pose.position.x, goal.pose.position.y,
                goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
                goal.pose.orientation.w)
            return
        if sign_dict[sign] is "NA":
            wall_count = 0
            goal.pose.position.y = cur_pos[1]
            goal.pose.position.x = cur_pos[0]

        elif sign_dict[sign] is "Goal":
            lost = False
            wall_count = 0
            rospy.loginfo("Arrive Final Goal!")
            while True:
                continue
            # return
        elif sign_dict[sign] is "Left":
            lost = False
            wall_count = 0
            dist[1] -= goal_offset
            # set goal position
            if cur_heading == 0 or cur_heading == 2:  # left/right
                goal.pose.position.x = cur_pos[0] + dist[1] if cur_heading == 2 else cur_pos[0] - dist[1]
                goal.pose.position.y = cur_pos[1]
            else:  # forward/backward
                goal.pose.position.y = cur_pos[1] + dist[1] if cur_heading == 1 else cur_pos[1] - dist[1]
                goal.pose.position.x = cur_pos[0]
            
            # set goal heading direction
            cur_heading = 3 if cur_heading - 1 < 0 else cur_heading - 1
            # goal.pose.orientation = heading[cur_heading]
            
        elif sign_dict[sign] is "Right":
            wall_count = 0
            lost = False
            dist[2] -= goal_offset
            # set goal position
            if cur_heading == 0 or cur_heading == 2:  # left/right
                goal.pose.position.x = cur_pos[0] + dist[2] if cur_heading == 0 else cur_pos[0] - dist[2]
                goal.pose.position.y = cur_pos[1]
            else:  # forward/backward
                goal.pose.position.y = cur_pos[1] + dist[2] if cur_heading == 3 else cur_pos[1] - dist[2]
                goal.pose.position.x = cur_pos[0]

            # set goal heading direction
            cur_heading = 0 if cur_heading + 1 > 3 else cur_heading + 1
            # goal.pose.orientation = heading[cur_heading]

        elif sign_dict[sign] is "DoNotEnter" or sign_dict[sign] is "Stop":
            wall_count = 0
            lost = False
            dist[3] -= goal_offset
            # set goal position
            if cur_heading == 0 or cur_heading == 2:  # left/right
                goal.pose.position.x = cur_pos[0]
                goal.pose.position.y = cur_pos[1] - dist[3] if cur_heading == 0 else cur_pos[1] + dist[3]
            else:  # forward/backward
                goal.pose.position.x = cur_pos[0] - dist[3] if cur_heading == 1 else cur_pos[0] + dist[3]
                goal.pose.position.y = cur_pos[1]
            # set goal heading direction
            if cur_heading == 1:
                cur_heading = 3
            elif cur_heading == 3:
                cur_heading = 1
            elif cur_heading == 0:
                cur_heading = 2
            elif cur_heading == 2:
                cur_heading = 0
            # goal.pose.orientation = heading[cur_heading]

        # elif sign_dict[sign] is "Stop":
        #     goal.pose.position.x = cur_pos[0]
        #     goal.pose.position.y = cur_pos[1]
        #     lost = False
        #     wall_count = 0
        
        elif sign_dict[sign] is "Wall":
            # No sign is shown
            wall_count += 1
            if wall_count > 8:
                if not lost:
                    start_time = time.time()
                lost = True
                rospy.loginfo_throttle(1, "Implementing Scan search")
                goal.pose.position.x = cur_pos[0]
                goal.pose.position.y = cur_pos[1]
                new_heading[0] = heading[cur_heading][0]
                new_heading[1] = heading[cur_heading][1]
                yaw = 0
                yaw = qua2rpy(heading[cur_heading])
                yaw = (yaw / 180) * math.pi
                if time.time() - start_time > 1:
                    if lost_count % 2 == 0:
                        yaw = yaw + ((lost_count * scan_angle_inc) + scan_angle_inc)
                    else:
                        yaw = yaw - ((lost_count * scan_angle_inc) + scan_angle_inc)
                    lost_count += 1
                rospy.loginfo_throttle(1, "Scan angle = %f", yaw)
                
                new_heading = euler_to_quaternion(0, 0, yaw)
        
        if not lost:
            lost_count = 0
            goal.pose.orientation.x = heading[cur_heading][0]
            goal.pose.orientation.y = heading[cur_heading][1]
            goal.pose.orientation.z = heading[cur_heading][2]
            goal.pose.orientation.w = heading[cur_heading][3]
        elif lost and any(new_heading) < 9000:
            if (time.time() - start_time > 1):
                start_time = time.time()
                goal.pose.orientation.x = new_heading[0]
                goal.pose.orientation.y = new_heading[1]
                goal.pose.orientation.z = new_heading[2]
                goal.pose.orientation.w = new_heading[3]

        rospy.loginfo("New Goal: (%f, %f), (%f,%f,%f,%f)", goal.pose.position.x, goal.pose.position.y,
                      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
                      goal.pose.orientation.w)


"""
Publish goal in 1HZ
"""


def pub_goal():

    global cur_pos
    global cur_ang
    listener = tf.TransformListener()
    rate = rospy.Rate(2) # prev value = 0.5
    # start_time = time.time()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            cur_pos[0] = trans[0]
            cur_pos[1] = trans[1]
            cur_pos[2] = trans[2]
            cur_ang = qua2rpy(rot)
            # rospy.loginfo("%f %f %f", cur_pos[0], cur_pos[1], cur_pos[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # if (time.time() - start_time > 2):
        #     start_time = time.time()
        set_goal()
        pub.publish(goal)
        rate.sleep()


""" Get distance to wall """


def obs_callback(msg):
    # msg = Wall()
    # float32 front
    # float32 left
    # float32 right
    # float32 back

    global dist_to_wall
    dist_to_wall[0] = msg.front
    dist_to_wall[1] = msg.left
    dist_to_wall[2] = msg.right
    dist_to_wall[3] = msg.back
    for i in range(4):
        if dist_to_wall[i] > 9999:
            dist_to_wall[i] = 0
    # if DEBUG:
    # rospy.loginfo_throttle(1, "Distance to wall = %2.2f m", dist_to_wall[1])


""" Get compressed image and predict sign """


def predict_image(CompressedImage):
    global sign
    imgBGR = bridge.compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
    # Predict sign here
    # imageBGR = np.reshape(imageBGR (410, 308, 3))
    sign = knn.predict(imgBGR, debug=DEBUG)
    rospy.loginfo_throttle(1, "%s Sign Detected", sign_dict[sign])
    # if DEBUG:
    # rospy.loginfo_throttle(1, "Sign predicted = %s", sign_dict[sign])


""" quaternion to euler angles """


def qua2rpy(qua):
    x = qua[0]
    y = qua[1]
    z = qua[2]
    w = qua[3]
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    # left -- yaw = 180-360
    # right -- yaw = 0-180
    yaw = y * 180 / math.pi
    if yaw < 0:
        yaw = yaw + 360

    return yaw

""" Euler to Quaternion conversion"""
def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]


""" get odometry data """


def odom_callback(msg):
    global cur_pos
    global cur_ang
    global initial
    rospy.sleep(0.5)
    # just run once because we cannot get robot initial position in the map frame
    if initial:
        cur_pos[0] = (msg.pose.pose.position.x)
        cur_pos[1] = (msg.pose.pose.position.y)
        cur_pos[2] = (msg.pose.pose.position.z)

        odom_qua = [0, 0, 0, 0]
        odom_qua[0] = (msg.pose.pose.orientation.x)
        odom_qua[1] = (msg.pose.pose.orientation.y)
        odom_qua[2] = (msg.pose.pose.orientation.z)
        odom_qua[3] = (msg.pose.pose.orientation.w)
        cur_ang = qua2rpy(odom_qua)
        
        set_goal()
        initial = False


""" update pose data in map frame"""


def map_callback(msg):
    global sign
    global cur_pos
    global cur_ang
    cur_pos[0] = (msg.feedback.base_position.pose.position.x)
    cur_pos[1] = (msg.feedback.base_position.pose.position.y)
    cur_pos[2] = (msg.feedback.base_position.pose.position.z)

    map_qua = [0, 0, 0, 0]
    map_qua[0] = (msg.feedback.base_position.pose.orientation.x)
    map_qua[1] = (msg.feedback.base_position.pose.orientation.y)
    map_qua[2] = (msg.feedback.base_position.pose.orientation.z)
    map_qua[3] = (msg.feedback.base_position.pose.orientation.w)
    cur_ang = qua2rpy(map_qua)


def init():
    # global cur_pos
    # global cur_ang
    global cur_heading
    ini_heading = rospy.get_param("/heading")
    if ini_heading == "left":
	cur_heading = 0
    elif ini_heading == "front":
	cur_heading = 1
    elif ini_heading == "right":
	cur_heading = 2
    elif ini_heading == "back":
	cur_heading = 3


    if sim:
        rospy.Subscriber("/turtlebot_burger/camera/image_raw/compressed", CompressedImage, predict_image, queue_size=1,
                         buff_size=2 ** 24)
    else:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, predict_image, queue_size=1,
                         buff_size=2 ** 24)
    rospy.Subscriber("/object_pos", Wall, obs_callback)
    # rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, map_callback)
    # rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.init_node('navigate_maze', anonymous=True)        
    
    # listener = tf.TransformListener()

    # try:
    #     (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    #     cur_pos[0] = trans[0]
    #     cur_pos[1] = trans[1]
    #     cur_pos[2] = trans[2]
    #     cur_ang = qua2rpy(rot)
    #     # rospy.loginfo("%f %f %f %f", rot[0], rot[1], rot[2], rot[3])
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     pass
    pub_goal()

    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
