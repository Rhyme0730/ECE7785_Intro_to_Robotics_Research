import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math


class nav2goal_node(Node):
    def __init__(self):
        super().__init__('nav2goal_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        self.sign_subscription = self.create_subscription(Int32,'/sign_state', self.get_sign,10)
        self.status_subscription = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.is_arrived, 10)
        self.lidat_subscription = self.create_subscription(LaserScan, '/scan', self.dist2wall, qos_profile)
        self.pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(0.2, self.nav2goal)

        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal_pose = self.goal.pose
        self.feedback = NavigateToPose.Feedback()

        self.sign = -1
        self.wall_front_flag = False

        self.waypoints = []  # TODO: find the discreter points in map
        self.curr_wp_idx = -1
        self.next_wp_idx = -1
        self.move_flag = False

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
            temp_dist = math.hypot(self.waypoints[i][0]-x, self.waypoints[i][1]-y)
            if temp_dist < min_dist:
                idx = i
                min_dist = temp_dist

        self.get_logger().info(f'the closest point to robot is {idx}, dist = {min_dist}')
        return idx
    
    def dist2wall(self, msg):
        '''
        Description: get the wall distance in front of robot (optional)
        '''
        self.wall_front_flag = True

    def get_sign(self, msg):
        '''
        Description: get the current sign on wall, notice that we need to check if the wall is front of robot
        '''       
        self.sign = msg.data
        if self.move_flag or not self.wall_front_flag:
            self.sign = None
            return
        self.get_logger().info(f'Robot received sign {self.sign}')

    def is_arrived(self, msg):
        '''
        Description: Check if arrived
        '''
        for status in msg.status_list:
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                self.feedback.current_pose.pose.orientation.w = self.goal_pose.orientation.w
                self.feedback.current_pose.pose.orientation.x = self.goal_pose.orientation.x
                self.feedback.current_pose.pose.orientation.y = self.goal_pose.orientation.y
                self.feedback.current_pose.pose.orientation.z = self.goal_pose.orientation.z

                self.get_logger().info(f'Arrived goal {self.next_wp_idx}')
                self.curr_wp_idx = self.next_wp_idx
                self.move_flag = False              
            elif status.status == GoalStatus.STATUS_ABORTED or status.status == GoalStatus.STATUS_CANCELLED:
                self.move_flag = False
                self.get_logger().info(f'Goal {self.next_wp_idx} cannot be reached')
                
    def is_arrived_v2(self):
        '''
        Description: Check whether robot is arrived, and update wp_idx
        '''
        dist2goal = np.array([self.goal_pose.position.x-self.feedback.current_pose.pose.position.x,
                              self.goal_pose.position.y-self.feedback.current_pose.pose.position.y])
        
        if np.abs(dist2goal[0]) < 0.1 and np.abs(dist2goal[1]) < 0.1:
            self.feedback.current_pose.pose.orientation.w = self.goal_pose.orientation.w
            self.feedback.current_pose.pose.orientation.x = self.goal_pose.orientation.x
            self.feedback.current_pose.pose.orientation.y = self.goal_pose.orientation.y
            self.feedback.current_pose.pose.orientation.z = self.goal_pose.orientation.z

            self.get_logger().info(f'Arrived goal {self.next_wp_idx}')
            self.curr_wp_idx = self.next_wp_idx
            self.move_flag = True
            return True
        else:
            return False

    def nav2goal(self):
        '''
        Description: navigate to discrete waypoints depend on different waypoints
        '''
        if self.curr_wp_idx == -1:
            self.next_wp_idx = self.closest_wp()
            goal_x = self.waypoints[self.next_wp_idx][0]
            goal_y = self.waypoints[self.next_wp_idx][1]
            self.goal_pose.position.x = goal_x
            self.goal_pose.position.y = goal_y

            self.get_logger().info(f'Navigating to closest waypoint index = {self.next_wp_idx}, pos = {goal_x, goal_y}')

        # TODO: other points sequence.
        if self.curr_wp_idx == 0:
            pass
        
        if self.curr_wp_idx == 1:
            pass
        
        if self.sign == 5:
            self.get_logger().info(f'---------------Finish navigating-----------------')

        arrive_flag = self.is_arrived()
        self.pose_publisher.publish(self.goal_pose)
    

def main(args=None):
    rclpy.init(args=args)
    nav2goal_n1 = nav2goal_node()
    rclpy.spin(nav2goal_n1)
    nav2goal_n1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





