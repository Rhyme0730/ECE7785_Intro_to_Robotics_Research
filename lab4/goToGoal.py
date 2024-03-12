#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from time import sleep

import numpy as np
import math

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0: return 0
    return 1 if val > 0 else 2

def on_segment(p, q, r):
    return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and 
            q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

def do_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
 
    if o1 != o2 and o3 != o4:
        return True
 
    if o1 == 0 and on_segment(p1, p2, q1): return True
    if o2 == 0 and on_segment(p1, q2, q1): return True
    if o3 == 0 and on_segment(p2, p1, q2): return True
    if o4 == 0 and on_segment(p2, q1, q2): return True
 
    return False

def segment_point_dist(x1, y1, x2, y2, x3, y3): # x3,y3 is the point
        px = x2-x1
        py = y2-y1

        norm = px*px + py*py

        u =  ((x3 - x1) * px + (y3 - y1) * py) / norm

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = x1 + u * px
        y = y1 + u * py

        dx = x - x3
        dy = y - y3

        dist = (dx*dx + dy*dy)**.5

        return dist

def pca(X):
    # Calculate the mean of the data
    mean_X = np.mean(X, axis=0)
    # Center the data 
    centered_X = X - mean_X
    # covariance matrix
    cov_matrix = np.cov(centered_X, rowvar=False)
    # eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    # Sort in descending order
    sorted_indices = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[sorted_indices]
    eigenvectors = eigenvectors[:, sorted_indices]
    
    return eigenvalues, eigenvectors, mean_X

class GoalGoer(Node):
    def __init__(self):
        super().__init__("goal_goer")

        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        
        self.object_subscriber = self.create_subscription(
            Float32MultiArray, 
            "/closest_object",
            self.object_callback,
            qos_profile)
        self.object_subscriber

        # State for the update_Odometry 
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        self.odom_sub  
        self.move_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # global frame x, y, theta
        self.g_WR = np.zeros(3)
        # world frame
        self.goals = np.array([[1.5, 0], [1.5, 1.4], [0, 1.4]])
        self.goal_index = 0
        self.current_goal = self.goals[self.goal_index]

        self.width = 0.2
        self.subgoal = None
        self.blocked = False
        self.direction = np.zeros(2)
                
        self.rotate = True
        self.edge = False
        self.rotated_90 = False
        self.u = None

        self.error_d = 0 
        self.error_a = 0 
        self.prev_error_d = 0
        self.prev_error_a = 0

        self.P_d = 0.5
        self.P_a = 0.7
        
        self.I_d = 0
        self.I_a = 0
        self.a_max = 1.0
        self.d_max = 1.0
        
        self.D_d = 0
        self.D_a = 0

        self.sum_d = 0
        self.sum_a = 0
    
    def set_goal(self, goal):
        self.current_goal = goal

    def to_point(self, angle, distance):
        x = distance*np.cos(angle)
        y = distance*np.sin(angle)
        return (x,y)
    
    def distance(self, p1, p2):
        return np.linalg.norm([p1[0]-p2[0], p1[1]-p2[1]])
    
    def get_position(self):
        return self.g_WR[:2]
    
    def frame(self, x, y, theta):
        M = np.array([[np.cos(theta), -np.sin(theta), x],
                     [np.sin(theta), np.cos(theta), y],
                     [0,0,1]])
        return M
        
    def goal_to_robot_frame(self, g):
        g_Wgoal = self.frame(g[0], g[1], 0)
        g_RW = np.linalg.inv(self.frame(self.g_WR[0], self.g_WR[1], self.g_WR[2]))
        g_Rgoal = g_RW@g_Wgoal
        x_Rgoal = g_Rgoal[0,2]
        y_Rgoal = g_Rgoal[1,2]
        goal = np.array([x_Rgoal, y_Rgoal])
        return goal
    
    def isBlocked(self, xs, ys):
        pos = [0,0]        
        goal = self.goal_to_robot_frame(self.current_goal)

        for i in range(len(xs)-1):
            p2 = [xs[i], ys[i]]
            q2 = [xs[i+1], ys[i+1]]
            if do_intersect(pos, goal, p2, q2):
                return True
            
        p2 = [xs[0], ys[0]]
        q2 = [xs[-1], ys[-1]]
        if do_intersect(pos, goal, p2, q2):
            return True

        for x,y in zip(xs,ys):
            if segment_point_dist(0,0, goal[0], goal[1], x,y)<self.width:
                return True

        return False

    def object_callback(self, msg: Float32MultiArray):
        data = msg.data
        a = np.array(data[0:len(data)//2]).reshape(-1,1)
        r = np.array(data[len(data)//2:]).reshape(-1,1)
        a_r = np.hstack((a,r))
        a = a_r[:,0]
        r = a_r[:,1]
        xs, ys = self.to_point(a,r)

        self.blocked = self.isBlocked(xs,ys)

        if not self.blocked:
            self.subgoal = None
        else:
            xy = np.array([xs,ys]).T
            left_point = xy[0,:]
            right_point = xy[-1,:]
            _, eigenvectors, _ = pca(xy)
            self.direction = eigenvectors[:,0]

            closest_point_idx = np.argmin(r) 
            self.subgoal = xy[closest_point_idx,:]

        return self.subgoal

    def move_to_(self, goal):
        # goal in robot frame
        self.prev_error_a = self.error_a
        self.prev_error_d = self.error_d
        if self.subgoal is None:
            goal = self.goal_to_robot_frame(goal)
        error_a = np.arctan2(goal[1], goal[0])
        self.error_a = (error_a+np.pi)%(2*np.pi)-np.pi
        self.error_d = np.linalg.norm(goal)

        if (abs(self.error_d)>0.1 and abs(self.error_a) < 0.3) or abs(self.error_a)<0.1:
            self.sum_d += self.error_d*self.dt
            diff_d = (self.error_d - self.prev_error_d)/self.dt
            self.u_d = self.P_d*self.error_d + self.I_d*self.sum_d + self.D_d*diff_d
            if abs(self.u_d)>0.2:
                self.u_d = 0.2*self.u_d/abs(self.u_d)
            msg = Twist()
            msg.linear.x = self.u_d 
        else:
            self.sum_a += self.error_a*self.dt
            diff_a = (self.error_a - self.prev_error_a)/self.dt
            self.u_a = self.P_a*self.error_a + self.I_a*self.sum_a + self.D_a*diff_a
            if abs(self.u_a)>1.5:
                self.u_a = 1.5*self.u_a/abs(self.u_a)
            msg = Twist()
            msg.angular.z = self.u_a
        
        self.move_publisher.publish(msg)

    def edge_travel(self):
        GR = self.goal_to_robot_frame(self.current_goal) 
        choice = np.argmax([GR.dot(self.direction), GR.dot(-1*self.direction)]) 
        if choice==0:
            self.u = self.direction
        else:
            self.u = -1*self.direction

        e_a = np.arctan2(self.u[1], self.u[0])
        e_a = (e_a+np.pi)%(2*np.pi)-np.pi
        print('ea: '+ str(e_a))
        print(self.u)
        if not self.rotate or abs(e_a)<0.2:
            # travel along edge
            self.rotate = False
            msg = Twist()
            msg.linear.x = 0.1
        elif self.rotate:
            # rotate till parallel with edge
            msg = Twist()
            msg.angular.z = self.P_a*e_a
        self.move_publisher.publish(msg)

    def timer_callback(self):
    """
    Move toward object until x distance away, use PCA to  travel parallel to the object. 
    Once all points are behind robot, turn toward goal. If path is free, continue to move to current goal 

    """
        if self.subgoal is None:
            print(4)
            # move to goal if nothing is in the way
            self.move_to_(self.current_goal)

            self.rotate = True
            self.edge = False
            self.rotated_90 = False
            self.u = None

        elif self.rotated_90:
            # go foward until the self.subgoal is None, clear path
            print(1)
            msg = Twist()
            msg.linear.x = 0.1
            self.move_publisher.publish(msg)

        elif self.edge or np.linalg.norm(self.subgoal)<self.width*2:
            print(2)
            # now travel along the edge
            self.edge_travel()
            self.edge = True

            if np.linalg.norm(self.subgoal) > 2*self.width:
                self.rotated_90 = True
         
        else:
            # move to subgoal first to move around object (the closest point)
            print(3)
            self.move_to_(self.subgoal)

        # reached goal, stay within in zone for 10 seconds
        if np.linalg.norm(self.current_goal-self.get_position())<0.02:
            sleep(5)
            self.goal_index += 1

            print(self.get_position())
            if self.goal_index==len(self.goals):
                rclpy.logging.get_logger("Goal Goer Node Info...").info("Finished!")
                rclpy.logging.get_logger("Goal Goer Node Info...").info(str(self.get_position()))
                self.destroy_node()
                rclpy.shutdown()

            self.current_goal = self.goals[self.goal_index]

    def odom_callback(self, data):
        self.update_Odometry(data)

    def update_Odometry(self,Odom: Odometry):
        position = Odom.pose.pose.position
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        self.g_WR = np.array([self.globalPos.x,  self.globalPos.y, self.globalAng])

def main():
    rclpy.init()
    goalgoer = GoalGoer()

    try:
        rclpy.spin(goalgoer)
    except SystemExit:
        rclpy.logging.get_logger("Goal Goer Node Info...").info("Shutting Down")

    goalgoer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()