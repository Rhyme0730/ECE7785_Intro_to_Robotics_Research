import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Vector3, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import numpy as np
import time

class goToGoal(Node):
    def __init__(self):
        super().__init__('goToGoal')
        self.get_logger().info("Starting goToGoal Node.................................")

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

        # Used for switching status
        self.flag = 0
        self.avoidFlag = False

        # Some params
        self.pos_Threshold = 0.08
        self.ang_Threshold = 0.03
        self.way_Point = np.array([[1.5, 0], [1.5, 1.4], [0, 1.4]])
        self.goal_ang = np.array([0.0, np.pi / 2, np.pi])

        # LIDAR msg
        self.d = 0.0
        self.theta = 0.0

        # Subscriber 1: Get robots global position from on board odometry sensor
        self.odom_Subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        # Subscriber 2: Detect whether there exist obstacles
        self.LIDAR_Subscriber = self.create_subscription(Point, '/closest_point', self.LIDAR_callback, 1)

        # Publisher 1: Publish Twist msg to cmd_vel
        self.vel_Publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd = Twist()

    ''' Get odom data and runRobot '''
    def odom_callback(self, odom_msg):
        self.update_Odometry(odom_msg)
        # self.get_logger().info(f'get robots global position x = {self.globalPos.x}, y = {self.globalPos.y} and global angle a = {self.globalAng}')
        self.run_Robot()

    ''' State Machine'''
    def run_Robot(self):
        # If finish tasks
        if self.flag == 4:
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            return

        # Detect obstacle
        if self.d <= 0.6:
            self.avoidFlag = True
            # self.get_logger().info(f'Avoiding the obstacle at d = {self.d}, theta = {self.theta}')
        else:
            self.avoidFlag = False
            # self.get_logger().info(f'Robot is tracking')

        # State Machine
        if self.avoidFlag:
            self.avoid_Obs()
        else:
            self.tracking()

    ''' Avoid obstacle '''
    def avoid_Obs(self):
        error_a = self.globalAng - self.goal_ang[self.flag] + 1/2*np.pi
        self.cmd.linear.x = 0.05*(1-np.exp(-self.d))  # The closer obstacle, the slower robot speed
        self.cmd.angular.z = self.controller(error_a, 1.2, 1.5)  # Turn to another direction

        self.vel_Publisher.publish(self.cmd)

    ''' Tracking waypoints'''
    def tracking(self):
        error_xy = np.array([self.globalPos.x - self.way_Point[self.flag][0], self.globalPos.y - self.way_Point[self.flag][1]])
 
        error_a = self.globalAng - self.goal_ang[self.flag]

        # Mean squared error
        MSE = np.sqrt(error_xy[0] ** 2 + error_xy[1] ** 2)

        # follow the way points
        flag_way = np.mod(self.flag, 2)  # 0: x axis, 1: y axis

        # moving along the y-axis
        if self.flag == 1:
            self.cmd.linear.x = self.controller(error_xy[flag_way], 1, 0.1)
            if error_xy[1-flag_way] >= 0.3:
                self.cmd.linear.x = 0.0
                error_a -= np.arctan((self.globalPos.x-1.5)/(1.4-self.globalPos.y))
                if error_a < self.ang_Threshold:
                    self.cmd.linear.x = self.controller(error_xy[flag_way], 1, 0.1)
                    self.cmd.angular.z = self.controller(error_a, 1.2, 1.5)

        # moving along x-axis            
        elif self.flag == 2:
            # self.get_logger().info(f'Robot is tracking y-axis')
            self.cmd.linear.x = -self.controller(error_xy[flag_way], 1, 0.1)

        # moving along the x-axis
        else:
            self.cmd.linear.x = self.controller(error_xy[flag_way], 1, 0.1)
            self.cmd.angular.z = self.controller(error_a, 1.2, 1.5)

        self.cmd.linear.y = self.controller(error_xy[1 - flag_way], 1, 0.1)
        # self.get_logger().info(f'x command velocity = {self.cmd.linear.x}')
        # self.get_logger().info(f'y command velocity = {self.cmd.linear.y}')

        # If angle error is large, then stop and turn the angle


        self.get_logger().info(f'error angle = {error_a}')
        # moving towards goal's angle
        # self.cmd.angular.z = self.controller(error_a, 1.2, 1.5)

        # If error of angle < 0.03, then stop turn the angle
        if np.abs(error_a) <= self.ang_Threshold:  # Todo: need to consider the case of angle >= 360
            self.cmd.angular.z = 0.0

        # If reached waypoint then stop and turn left
        if MSE <= self.pos_Threshold:
            self.cmd.linear.x = 0.0
            self.turn_left()

        self.vel_Publisher.publish(self.cmd)

    ''' Turn robot to left after reaching way point'''
    def turn_left(self):
        # Turn to face next way point
        error_a = self.globalAng - self.goal_ang[self.flag + 1]
        self.cmd.angular.z = self.controller(error_a, 0.5, 1.5)
        if np.abs(error_a) <= self.ang_Threshold:
            self.get_logger().info("Got you! you are going to turn left")
            self.cmd.angular.z = 0.0
            # self.sleep_robot()
            self.get_logger().info("Start to sleep for 10 seconds..............")

            if self.flag <= 1:
                self.flag += 1  # Go to next point

    ''' Get LIDAR data '''
    def LIDAR_callback(self, lidar_msg):
        self.theta = lidar_msg.x * 360.0
        self.d = lidar_msg.y

    ''' Currently only P control'''
    # PID controller
    def controller(self, error, Kp, max):  # Todo: upgrade to PID controller
        u = self.limit(-Kp * error, max)
        return u

    ''' Limit function '''
    def limit(self, val, max):
        if val >= max:
            val = max
        elif val <= -max:
            val = -max
        return val

    ''' Sleep for 10 seconds '''
    def sleep_robot(self):
        self.get_logger().info('Pausing for 10 seconds.........')
        time.sleep(10)
        self.get_logger().info('Times now.........')

    ''' Update the odom data in global frame'''
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
    gTG = goToGoal()
    rclpy.spin(gTG)
    gTG.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
