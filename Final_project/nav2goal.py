import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
import numpy as np


class nav2goal:
    def __init__(self):
        super().__init__('nav2goal')
        self.subscription = self.create_subscription(Int32,'/sign_state',self.state_callback,10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.status_subscription = self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.goal_reached_callback,10)
        
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"

        self.timer = self.create_timer(0.2, self.navigate_to_waypoint)

    
    def 
