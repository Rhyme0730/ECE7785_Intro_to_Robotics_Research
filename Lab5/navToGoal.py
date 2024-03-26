import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Vector3, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import time 

class nav_to_goal(Node):
    def __init__(self):
        super().__init__('navToGoal')
        self.get_logger().info("Starting navToGoal Node.................................")
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Used for printing the odometry data
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

        way_Point_1 = np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        way_Point_2 = np.array([2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        way_Point_3 = np.array([3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        # way_Point_1 = np.array([1.0, 1.0, 1.0])
        # way_Point_2 = np.array([1.0, 1.5, 2.0])
        # way_Point_3 = np.array([2.0, 1.0, 3.0])
        self.wayPoint = np.array([way_Point_1, way_Point_2, way_Point_3])

        self.flag = 0

        self.x = 0.0
        self.y = 0.0


        self.goal = PoseStamped()

        self.odom_Subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        self.nav_Publisher = self.create_publisher(PoseStamped, '/goal_pose',qos_profile)
    
    def odom_callback(self, odom_msg):
        # self.update_Odometry(odom_msg)
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        # self.get_logger().info(f'x={self.globalPos.x}, y={self.globalPos.y}, ang={self.globalAng}')
        self.update_goal()

    def update_goal(self):
        if self.isReached():
            self.get_logger().info(f'robot has reached goal {self.flag}')
            if self.flag >= 3:
                self.get_logger().info('****************Finish***************')

        self.goal.header.frame_id = 'map'
        self.goal.pose.position.x = self.wayPoint[self.flag][0]
        self.goal.pose.position.y = self.wayPoint[self.flag][1]
        self.goal.pose.orientation.x = self.wayPoint[self.flag][3]
        self.goal.pose.orientation.y = self.wayPoint[self.flag][4]
        self.goal.pose.orientation.z = self.wayPoint[self.flag][5]
        self.goal.pose.orientation.w = self.wayPoint[self.flag][6]

        self.nav_Publisher.publish(self.goal)


    def isReached(self):
        if np.abs(self.wayPoint[self.flag][0] - self.x) < 0.1 and np.abs(self.wayPoint[self.flag][1] - self.y) < 0.1:
            self.flag += 1
            return True
        else:
            return False
        

    def update_Odometry(self, Odom):
            position = Odom.pose.pose.position
            # Orientation uses the quaternion aprametrization.
            # To get the angular position along the z-axis, the following equation is required.
            q = Odom.pose.pose.orientation
            orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            if self.Init:
                # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
                self.Init = False
                self.Init_ang = orientation
                self.globalAng = self.Init_ang
                Mrot = np.matrix(
                    [[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
                self.Init_pos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y
                self.Init_pos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y
                self.Init_pos.z = position.z
            Mrot = np.matrix(
                [[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])

            # We subtract the initial values
            self.globalPos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y - self.Init_pos.x
            self.globalPos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y - self.Init_pos.y
            self.globalAng = orientation - self.Init_ang        

def main(args=None):
    rclpy.init(args=args)
    nTG = nav_to_goal()
    rclpy.spin(nTG)
    nTG.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
