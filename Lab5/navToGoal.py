import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from nav_msgs.msg import Odometry
import numpy as np
import time

class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")

        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        self.FB_subscriber = self.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback', self.FB_callback, qos_profile)
        self.odom_Subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.current_i = 0

        self.goal = PoseStamped()
        way_Point_1 = np.array([0.5, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0])
        way_Point_2 = np.array([1.5, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        way_Point_3 = np.array([3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.wayPoint = np.array([way_Point_1, way_Point_2, way_Point_3])

        self.pos1 = NavigateToPose_FeedbackMessage()

    def odom_callback(self, msg):

        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = self.wayPoint[self.current_i][0]
        self.goal.pose.position.y = self.wayPoint[self.current_i][1]
        self.goal.pose.orientation.x = self.wayPoint[self.current_i][3]
        self.goal.pose.orientation.y = self.wayPoint[self.current_i][4]
        self.goal.pose.orientation.z = self.wayPoint[self.current_i][5]
        self.goal.pose.orientation.w = self.wayPoint[self.current_i][6]

        x1, y1 = self.pos1.feedback.current_pose.pose.position.x, self.pos1.feedback.current_pose.pose.position.y
        pos2 = self.goal.pose.position
        x2, y2 = pos2.x, pos2.y
        dx = x1-x2
        dy = y1-y2
        if np.linalg.norm([dx, dy]) < 0.2:
            self.current_i += 1
            print(self.current_i)
            if self.current_i >= 3:
                print('finish')
                return
        self.goal_pose_publisher.publish(self.goal)


    def FB_callback(self, msg:NavigateToPose_FeedbackMessage):
       self.pos1 = msg

def main(args=None):
    rclpy.init(args=args)
    nTG = Navigator()
    rclpy.spin(nTG)
    nTG.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
