#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import numpy as np
import time

class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
  ses nav stack to move around 
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.FB_subscriber = self.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback', self.FB_callback, qos_profile)
        self.FB_subscriber
        
        # POINTS SET1
        # x1, y1 = 1.5, 0.5
        # x2, y2 = 2.5, -1
        # x3, y3 = 3, 0.5
        
        # POINTS SET2
        x1, y1 = -0.16, -0.7
        x2, y2 = 0.86, -0.7
        x3, y3 = 1.4, 0
        self.waypoints = [
            PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(x=float(x1), y=float(y1)))),
            PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(x=0.0, y=0.7))),
            PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(x=float(x2), y=float(y2)))),
            PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(x=float(x3), y=float(y3)))),
        ]
        self.current_i = 0
        print('hello')
        time.sleep(0.5)
        self.goal_pose_publisher.publish(self.waypoints[self.current_i])

    def FB_callback(self, msg: NavigateToPose_FeedbackMessage):
        print(self.current_i)
        pos1 = msg.feedback.current_pose.pose.position
        x1, y1 = pos1.x, pos1.y
        pos2 = self.waypoints[self.current_i].pose.position
        x2, y2 = pos2.x, pos2.y
        dx = x1-x2
        dy = y1-y2
        if np.linalg.norm([dx, dy]) < 0.1:
            self.current_i += 1
            if self.current_i>=len(self.waypoints):
                print('finish')
                return
            self.goal_pose_publisher.publish(self.waypoints[self.current_i])

def main():
    rclpy.init()
    nav = Navigator()

    try:
        rclpy.spin(nav)
    except SystemExit:
        rclpy.logging.get_logger("Object Follower Node Info...").info("Shutting Down")
    
    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()