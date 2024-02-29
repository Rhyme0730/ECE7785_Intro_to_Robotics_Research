import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Vector3, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import numpy as np 

class goToGoal(Node):
    def __init__(self):
        super().__init__('goToGoal')
        self.get_logger().info("Starting goToGoal Node.................................")


        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

        # Used for switching status
        self.flag = 1
        self.sleep_flag = False
        self.turn_index = 0

        # some params
        self.pos_Threshold = 0.01
        self.ang_Threshold = 0.01
        self.way_Point_1 = np.array([1.5, 0])
        self.way_Point_2 = np.array([1.5, 1.4])
        self.way_Point_3 = np.array([1.5, 1.4])
        self.goal_ang = np.array([0.0, np.pi/2, np.pi])

        
        # Subscriber 1: Get robots global position from on board odometry sensor
        self.odom_Subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        # Subscriber 2: Detect whether there exist obstacles

        # Publisher 1: Publish Twist msg to cmd_vel
        self.vel_Publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.rate = self.create_rate(0.1)
        self.cmd = Twist()

    def odom_callback(self, odom_msg):
        self.update_Odometry(odom_msg)
        # self.get_logger().info(f'get robots global position x = {self.globalPos.x}, y = {self.globalPos.y} and global angle a = {self.globalAng}')
        self.simple_run()

    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

    def simple_run(self):
        match self.flag:
            case 1: ## follow the way point
                error_x = self.globalPos.x - self.way_Point_1[0]
                error_y = self.globalPos.y - self.way_Point_1[1]
                error_a = self.globalAng - self.goal_ang[0]
                self.cmd.linear.x = self.controller(error_x, 1, 0.2)
                
                if error_a <= np.pi and error_a >= -np.pi:
                    self.cmd.angular.z = self.controller(error_a, 0.5, 1.5)

                if np.abs(error_a) <= self.ang_Threshold:
                    self.cmd.angular.z = 0.0

                if np.abs(error_x) <= self.pos_Threshold:
                    self.cmd.linear.x = 0.0
                    self.turn_left(1)

                self.vel_Publisher.publish(self.cmd)


    def turn_left(self, index):
        error_a = self.globalAng - self.goal_ang[index]
        self.cmd.angular.z = self.controller(error_a, 0.5, 1.5)
        if np.abs(error_a) <= self.ang_Threshold:
            self.get_logger().info("Got you! you are going to turn left")
            self.cmd.angular.z = 0.0
            self.sleep_flag = True
        


    def controller(self, error, Kp, max):
        u = self.limit(-Kp*error,max)
        return u

        # Limit function
    def limit(self, val, max):
        if val >= max:
            val = max
        elif val <= -max:
            val = -max
        return val

    # Some problems....
    def sleep_robot(self):
        if self.sleep_flag == True:
            self.get_logger().info('Pausing for 10 seconds.........')
            self.rate.sleep()
            self.get_logger().info('Times now.........')
            self.sleep_flag = False


def main(args=None):
    rclpy.init(args=args)
    gTG = goToGoal()
    rclpy.spin(gTG)
    gTG.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
