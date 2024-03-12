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

        # Obstacles
        self.obs_xy_1 = np.array([0.0, 0.0])
        self.obs_xy_2 = np.array([0.0, 0.0])

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
        self.run_Robot()
        # self.tracking()
        # self.debug1()

    ''' State Machine'''
    def run_Robot(self):
        # If finish tasks
        if self.flag == 3:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            return

        # Detect obstacle
        if self.d <= 0.4 and self.d > 0.0:
            self.avoidFlag = True
            # self.get_logger().info(f'Avoiding the obstacle at d = {self.d}, theta = {self.theta}')
        else:
            self.avoidFlag = False

        # State Machine
        if self.avoidFlag:
            self.avoid_Obs()
        else:
            self.tracking()

    ''' Avoid obstacle '''
    def avoid_Obs(self):
        kp_A = 1.5
        kp_V = 1.0
        ang_Max = 0.5
        vel_Max = 0.2
        pos_Tol = 0.05
        ang_Tol = 0.05

        x_goal = self.way_Point[self.flag][0]
        y_goal = self.way_Point[self.flag][1]
        x = self.globalPos.x
        y = self.globalPos.y
        ang = self.globalAng
        theta = np.arctan((y_goal-y)/(x_goal-x))
        error_d = 0.0
        self.cmd.angular.z = 0.0
        if self.flag == 1:
            if theta < 0:
                theta += np.pi
            error_d = y_goal - y
            self.obs_xy_1 = np.array([x_goal,y+self.d]) # record the obstacle position
            self.cmd.angular.z = -0.2
        
        if self.flag == 2:
            if ang < 0:
                ang += 2*np.pi
            theta += np.pi
            error_d = -(x_goal - x)
            self.obs_xy_2 = np.array([x-self.d,y_goal]) # record the obstacle position 
            self.cmd.angular.z = 0.2

        self.cmd.linear.x = 0.0
        self.vel_Publisher.publish(self.cmd)

    def debug2(self):
        x_goal = self.way_Point[2][0]
        y_goal = self.way_Point[2][1]
        x = self.globalPos.x
        y = self.globalPos.y
        ang = self.globalAng
        if ang < 0:
            ang += 2*np.pi
        theta = np.arctan((y_goal-y)/(x_goal-x))
        theta += np.pi
        
        ang *= 180/np.pi
        theta *= 180/np.pi

        error_a = theta - ang

        self.get_logger().info(f'ang = {ang}, theta = {theta}, error_a = {error_a}')

    def debug1(self):
        x_goal = self.way_Point[1][0]
        y_goal = self.way_Point[1][1]
        x = self.globalPos.x
        y = self.globalPos.y
        ang = self.globalAng
        theta = np.arctan((y_goal-y)/(x_goal-x))
        error_d = y_goal - y

        if theta < 0:
            theta += np.pi

        error_a = theta - ang
        self.cmd.angular.z = self.controller(error_a,1.0,0.3)
        self.cmd.linear.x = self.controller(error_d,1.0,0.1)
        ang *= 180/np.pi
        theta *= 180/np.pi

        self.vel_Publisher.publish(self.cmd)

        

        self.get_logger().info(f'ang = {ang}, theta = {theta}, error_a = {error_a}')
    
    def debug0(self):
        x_goal = self.way_Point[0][0]
        y_goal = self.way_Point[0][1]
        x = self.globalPos.x
        y = self.globalPos.y
        ang = self.globalAng
        theta = np.arctan((y_goal-y)/(x_goal-x))
        
        ang *= 180/np.pi
        theta *= 180/np.pi

        error_a = theta - ang

        self.get_logger().info(f'ang = {ang}, theta = {theta}, error_a = {error_a}')

    def tracking(self):
        kp_A = 2.0
        kp_V = 1.0
        ang_Max = 0.4
        vel_Max = 0.15
        pos_Tol = 0.03
        ang_Tol_Min = 0.04
        ang_Tol_Max = 0.2

        x_goal = self.way_Point[self.flag][0]
        y_goal = self.way_Point[self.flag][1]
        x = self.globalPos.x
        y = self.globalPos.y
        ang = self.globalAng

        theta = np.arctan((y_goal-y)/(x_goal-x))
        error_d = 0.0
        error_a = 0.0

        if self.flag == 0:
            error_d = x_goal - x
            error_a = theta - ang

        if self.flag == 1:
            if theta < 0:
                theta += np.pi
            error_d = y_goal - y + 0.05
            error_a = theta - ang

            if self.obs_xy_1[0] != 0.0 and y < self.obs_xy_1[1]:
                error_a -= 0.4*np.pi
                self.get_logger().info(f'obstacle1 coordinates = {(self.obs_xy_1[0],self.obs_xy_1[1])}')
        
        if self.flag == 2:
            if ang < 0:
                ang += 2*np.pi
            theta += np.pi
            error_d = -(x_goal - x) + 0.08
            error_a = theta - ang

            if self.obs_xy_2[0] != 0.0 and x > self.obs_xy_2[0]:
                # error_a -= 0.4*np.pi
                error_a += 0.4*np.pi
                self.get_logger().info(f'obstacle2 coordinates = {(self.obs_xy_2[0],self.obs_xy_2[1])}')
            
        self.cmd.angular.z = self.controller(error_a, kp_A, ang_Max)
        self.cmd.linear.x = self.controller(error_d, kp_V, vel_Max)

        # self.get_logger().info(f'flag = {self.flag}, theta = {theta}, ang = {ang}, error_a = {error_a}, error_d = {error_d}, cmd.z = {self.cmd.angular.z}, cmd.x = {self.cmd.linear.x}')
        if np.abs(error_a) > ang_Tol_Max:
            self.cmd.linear.x = 0.0

        if np.abs(error_a) < ang_Tol_Min:
            self.cmd.angular.z = 0.0

        if np.abs(error_d) < pos_Tol:
            self.cmd.linear.x = 0.0
            self.flag += 1
            self.vel_Publisher.publish(self.cmd)
            self.sleep_robot()
        else:
            self.vel_Publisher.publish(self.cmd)

    ''' Get LIDAR data '''
    def LIDAR_callback(self, lidar_msg):
        self.theta = lidar_msg.x * 360.0
        self.d = lidar_msg.y

    ''' Currently only P control'''
    # PID controller
    def controller(self, error, Kp, max):  # Todo: upgrade to PID controller
        u = self.limit(Kp * error, max)
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




