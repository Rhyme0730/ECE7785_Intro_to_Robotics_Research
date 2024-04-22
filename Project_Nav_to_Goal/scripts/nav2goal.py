import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math
import time
from nav_msgs.msg import Odometry


class nav2goal_node(Node):
    def __init__(self):
        super().__init__('nav2goal_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        self.sign_subscription = self.create_subscription(Point, '/sign_state', self.get_sign, 10)
        self.status_subscription = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                                            self.is_arrived, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.dist2wall, qos_profile)
        # self.timer = self.create_timer(0.2, self.simple_test)
        self.odom_Subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.timer = self.create_timer(0.2, self.nav2goal)

        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal_pose = self.goal.pose
        self.feedback = NavigateToPose.Feedback()

        # need to change this start point use /clicked_point!!!!!!!!
        self.feedback.current_pose.pose.position.x = -0.42
        self.feedback.current_pose.pose.position.y = -0.81
        # self.feedback.current_pose.pose.orientation.x = 0.0
        # self.feedback.current_pose.pose.orientation.y = 0.0
        # self.feedback.current_pose.pose.orientation.z = 0.0
        # self.feedback.current_pose.pose.orientation.w = 0.0

        self.sign = -1
        self.wall_front_flag = False

        self.waypoints = np.array([[-3.22, 0.15],  # 0
                                   [-2.96, -0.79],  # 1
                                   [-4.02, -0.95],  # 2
                                   [-4.21, 0.00],  # 3
                                   [-4.12, 1.18],  # 4
                                   [-3.13, 1.17],  # 5
                                   [-2.15, 1.04],  # 6:goal
                                   [-2.12, 0.17],  # 7
                                   [-2.34, -0.79],  # 8
                                   [-1.31, -0.94],  # 9: right
                                   [-1.43, 0.13],  # 10: backward
                                   [-1.34, 1.00],  # 11: backward
                                   [-0.38, 1.14],  # 12: left
                                   [-0.44, -0.04],  # 13: right
                                   [0.49, 0.19],  # 14: left
                                   [0.64, -0.67],  # 15: forward
                                   [-0.38, -0.72]  # 16: left
                                   ])
        self.curr_wp_idx = -1
        self.next_wp_idx = -2
        self.move_flag = True
        self.achieve_flag = False

        self.left = [0.0, 0.0, 0.707, 0.707]
        self.right = [0.0, 0.0, -0.707, 0.707]
        self.forward = [0.0, 0.0, 0.0, 1.0]
        self.backward = [0.0, 0.0, 1.0, 0.0]

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0
        self.ori = [0.0, 0.0, 0.0, 0.0]

    def odom_callback(self, msg: Odometry):
        '''
        Use odom angle to check whether the turn action is achieved
        '''
        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

    def closest_wp(self):
        '''
        Description: return the closest waypoint distance to robot
        '''
        if self.feedback is None:
            return None

        idx = None
        min_dist = np.inf
        x = self.feedback.current_pose.pose.position.x
        y = self.feedback.current_pose.pose.position.y
        for i in range(len(self.waypoints)):
            temp_dist = math.hypot(self.waypoints[i][0] - x, self.waypoints[i][1] - y)
            if temp_dist < min_dist:
                idx = i
                min_dist = temp_dist

        # self.get_logger().info(f'the closest point to robot is {idx}, dist = {min_dist}')
        return idx

    def dist2wall(self, msg):
        '''
        Description: get the wall distance in front of robot (optional)
        '''
        self.wall_front_flag = True

    def get_sign(self, msg: Point):
        '''
        Description: get the current sign on wall, notice that we need to check if the wall is front of robot
        '''
        self.sign = msg.x
        if self.move_flag:
            self.sign = None
            return
        # print(f'Robot received sign {self.sign}')

    def LIDAR_callback(self, lidar_msg):
        self.theta = lidar_msg.x * 360.0
        self.d = lidar_msg.y

    def is_arrived(self, msg):
        '''
        Description: Check if arrived
        '''
        for status in msg.status_list:
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                # self.feedback.current_pose.pose.orientation.w = self.goal_pose.orientation.w
                # self.feedback.current_pose.pose.orientation.x = self.goal_pose.orientation.x
                # self.feedback.current_pose.pose.orientation.y = self.goal_pose.orientation.y
                # self.feedback.current_pose.pose.orientation.z = self.goal_pose.orientation.z
                self.move_flag = False
                # self.get_logger().info(f'Arrived goal {self.goal_pose.position}')
            elif status.status == GoalStatus.STATUS_ABORTED or status.status == GoalStatus.STATUS_CANCELED:
                self.move_flag = False
                # self.get_logger().info(f'Goal {self.next_wp_idx} cannot be reached')

    def simple_test(self):
        if self.next_wp_idx == -2:
            self.next_wp_idx = self.closest_wp()
        else:
            ori = self.forward

            if self.next_wp_idx == 4:
                print(f'4')
                ori = self.left
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 5

            elif self.next_wp_idx == 5:
                print(f'5')
                ori = self.left
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 6

            elif self.next_wp_idx == 3:
                print(f'3')
                ori = self.backward
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 2

            elif self.next_wp_idx == 2:
                print(f'2')
                ori = self.left
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 1

            elif self.next_wp_idx == 1:
                print(f'1')
                ori = self.forward
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 0

            elif self.next_wp_idx == 0:
                print(f'0')
                ori = self.right
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 7

            elif self.next_wp_idx == 8:
                print(f'8')
                ori = self.backward
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 9

            elif self.next_wp_idx == 7:
                print(f'7')
                ori = self.forward
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 6

            elif self.next_wp_idx == 6:
                print(f'6')
                ori = self.forward
                if self.check_ang(ori):
                    if self.sign == 5.0:
                        print('-----------finish----------')
                        # return
            elif self.next_wp_idx == 9:
                print(f'9')
                ori = self.forward
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 10

            elif self.next_wp_idx == 10:
                print(f'10')
                ori = self.backward
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 11

            elif self.next_wp_idx == 11:
                print(f'11')
                ori = self.backward
                if self.check_ang(ori):
                    if self.sign == 4.0:
                        self.next_wp_idx = 12

            elif self.next_wp_idx == 12:
                print(f'12')
                ori = self.forward
                if self.check_ang(ori):
                    if self.sign == 5.0:
                        print('finish goal')
                        return
                        # self.next_wp_idx = 11

            elif self.next_wp_idx == 13:
                print(f'13')
                ori = self.right
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 10

            elif self.next_wp_idx == 14:
                print(f'14')
                ori = self.left
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 10

            elif self.next_wp_idx == 15:
                print(f'15')
                ori = self.forward
                if self.check_ang(ori):
                    if self.sign == 1.0:
                        self.next_wp_idx = 14

            elif self.next_wp_idx == 16:
                print(f'16')
                ori = self.left
                if self.check_ang(ori):
                    if self.sign == 2.0:
                        self.next_wp_idx = 15

            self.goal.pose.orientation.x = ori[0]
            self.goal.pose.orientation.y = ori[1]
            self.goal.pose.orientation.z = ori[2]
            self.goal.pose.orientation.w = ori[3]

        self.goal.pose.position.x = self.waypoints[self.next_wp_idx][0]
        self.goal.pose.position.y = self.waypoints[self.next_wp_idx][1]
        self.pose_publisher.publish(self.goal)
        self.move_flag = True

    def nav2goal(self):
        '''
        Description: Decision Tree for robot navigating in maze
        '''
        if self.next_wp_idx == -2:  # Find the closest point
            self.next_wp_idx = self.closest_wp()
        else:
            ori = self.forward
            if self.next_wp_idx == 4:
                print(f'4')
                ori = self.left
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 3
                    elif self.sign == 2.0:
                        self.next_wp_idx = 5
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 3
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 5:
                print(f'5')
                ori = self.left
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 4
                    elif self.sign == 2.0:
                        self.next_wp_idx = 6
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 0
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 3:
                print(f'3')
                ori = self.backward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 2
                    elif self.sign == 2.0:
                        self.next_wp_idx = 4
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 0
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 2:
                print(f'2')
                ori = self.right
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 1
                    elif self.sign == 2.0:
                        self.ori = self.backward
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 3
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 1:
                print(f'1')
                ori = self.forward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 0
                    elif self.sign == 2.0:
                        self.ori = self.right
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 2
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 0:
                print(f'0')
                ori = self.right
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 3
                    elif self.sign == 2.0:
                        self.next_wp_idx = 7
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 1
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 8:
                print(f'8')
                ori = self.backward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.ori = self.right
                    elif self.sign == 2.0:
                        self.next_wp_idx = 7
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 9
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 7:
                print(f'7')
                ori = self.forward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 6
                    elif self.sign == 2.0:
                        self.next_wp_idx = 8
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 0
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 6:
                print(f'6')
                ori = self.forward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.ori = self.left
                    elif self.sign == 2.0:
                        self.next_wp_idx = 7
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 5
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 9:
                print(f'9')
                ori = self.forward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.ori = self.forward
                    elif self.sign == 2.0:
                        self.next_wp_idx = 8
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 10
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 10:
                print(f'10')
                ori = self.backward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 9
                    elif self.sign == 2.0:
                        self.next_wp_idx = 11
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 13
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 11:
                print(f'11')
                ori = self.backward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 10
                    elif self.sign == 2.0:
                        self.ori = self.left
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 12
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 12:
                print(f'12')
                ori = self.forward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                if self.sign == 1.0:
                    self.next_wp_idx = 11
                elif self.sign == 2.0:
                    self.ori = self.forward
                elif self.sign == 3.0 or self.sign == 4.0:
                    self.next_wp_idx = 13
                elif self.sign == 5.0:
                    return

            elif self.next_wp_idx == 13:
                print(f'13')
                ori = self.right
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 14
                    elif self.sign == 2.0:
                        self.next_wp_idx = 10
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 12
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 14:
                print(f'14')
                ori = self.left
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 13
                    elif self.sign == 2.0:
                        self.ori = self.forward
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 15
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 15:
                print(f'15')
                ori = self.forward
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.next_wp_idx = 14
                    elif self.sign == 2.0:
                        self.ori = self.right
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.next_wp_idx = 16
                    elif self.sign == 5.0:
                        return

            elif self.next_wp_idx == 16:
                print(f'16')
                ori = self.left
                if self.check_ang(ori):
                    self.get_action(self.sign)
                    if self.sign == 1.0:
                        self.ori = self.backward
                    elif self.sign == 2.0:
                        self.next_wp_idx = 15
                    elif self.sign == 3.0 or self.sign == 4.0:
                        self.ori = self.forward
                    elif self.sign == 5.0:
                        return

            self.goal.pose.orientation.x = ori[0]
            self.goal.pose.orientation.y = ori[1]
            self.goal.pose.orientation.z = ori[2]
            self.goal.pose.orientation.w = ori[3]

        self.goal.pose.position.x = self.waypoints[self.next_wp_idx][0]
        self.goal.pose.position.y = self.waypoints[self.next_wp_idx][1]
        self.pose_publisher.publish(self.goal)
        self.move_flag = True

    def check_ang(self, ori):
        '''
        Description: Check if desired angle is achieved
        '''
        x = self.orientation_x
        y = self.orientation_y
        z = self.orientation_z
        w = self.orientation_w
        curr_qua = [self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w]
        curr_ori = qua2eul(curr_qua)
        goal_ori = qua2eul(ori)
        print(f'curr_ang = {curr_ori}, goal_ang={goal_ori}')
        if np.abs(curr_ori - goal_ori) < 0.1:
            return True
        return False

    def get_action(self):
        '''
        Description: Detect the sign and do desired turn actions
        '''
        sign = self.sign
        if sign == 1.0:
            self.ori = self.left
        elif sign == 2.0:
            self.ori = self.right
        elif sign == 3.0 or sign == 4.0:
            self.ori == self.backward
        elif sign == 5.0:
            print('---------------Finish Task---------------------')
            return None


def qua2eul(q):
    '''
    Description: Transform quaternion to euler angle
    '''
    orientation = np.arctan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
    return orientation


def main(args=None):
    rclpy.init(args=args)
    nav2goal_n1 = nav2goal_node()
    rclpy.spin(nav2goal_n1)
    nav2goal_n1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





