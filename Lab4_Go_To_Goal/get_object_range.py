#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np

class get_object_range(Node):
    def __init__(self):
        super().__init__("get_object_range")

        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            "/scan",
            self.scan_callback,
            qos_profile)
        self.scan_subscriber
        
        self.displacement_publisher = self.create_publisher(Point, '/closest_point', 1)
        self.object_publisher = self.create_publisher(Float32MultiArray, '/closest_object', 1)
        
        self.epsilon = 1
        self.min_points = 5

    def to_point(self, angle, distance):
        x = distance*np.sin(angle)
        y = distance*np.cos(angle)
        return (x,y)

    def distance(self, p1, p2):
        return np.linalg.norm([p1[0]-p2[0], p1[1]-p2[1]])

    def scan_callback(self, scan: LaserScan):
        # group points into objects
        objects = {}
        object_id = 0
        prev_a = scan.angle_min
        prev_r = scan.ranges[0]
        for a, r in zip(np.arange(scan.angle_min, scan.angle_max, scan.angle_increment), scan.ranges):
            # if distance between ranges is close enough, group the ranges
            p1 = self.to_point(a,r)
            p2 = self.to_point(prev_a,prev_r)
            if self.distance(p1,p2)<self.epsilon:
                if object_id not in objects:
                    objects[object_id] = []
                objects[object_id].append([a,r])
            # new object with the new range
            else:
                object_id += 1
                objects[object_id] = []
                objects[object_id].append([a,r])
            prev_r = r
            prev_a = a
        
        # connect last and first range entries if they are the same object
        first_point = self.to_point(scan.angle_min, scan.ranges[0])
        last_point = self.to_point(scan.angle_max, scan.ranges[len(scan.ranges)-1])
        if self.distance(first_point, last_point)<self.epsilon and len(objects)>2:
            last_object = objects[object_id]
            objects[0] = last_object + objects[0]
            objects.pop(object_id)

        remove_idx = []
        for key in objects:
            if len(objects[key])<self.min_points:
                remove_idx.append(key)
        for idx in remove_idx:
            objects.pop(idx)

        # publish closest object
        min_angle = np.inf
        min_range = np.inf
        min_idx = -1
        for key in objects:
            for a, r in objects[key]:
                if r<min_angle:
                    min_angle = a
                    min_range = r
                    min_idx = key

        # publishes closest point distance
        msg = Point()
        msg.x, msg.y = self.to_point(min_angle, min_range)
        point_str = f"Point(x={msg.x}, y={msg.y}, z={msg.z})"
        self.displacement_publisher.publish(msg)
        self.get_logger().info(point_str)

        # publishes closest object
        msg = Float32MultiArray()
        aa, rr = [], []
        a_r = np.array(objects[min_idx]).astype('float')
        msg.data = a_r[:,0].tolist() + a_r[:,1].tolist()
        data_str = str(list(msg.data))
        self.object_publisher.publish(msg)
        # self.get_logger().info(data_str)

        
        return objects, msg


def main():
    rclpy.init()
    get_object_range1 = get_object_range()
    rclpy.spin(get_object_range1)
    get_object_range1.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()













# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
# from std_msgs.msg import String

# import numpy as np

# from geometry_msgs.msg import Point, Twist
# from sensor_msgs.msg import LaserScan


# class get_object_range(Node):
#     def __init__(self):
#         super().__init__('get_object_range')

#         self.angle = 0.0 # may need change

#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#             history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
#             durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
#             depth=1
#         )
#         self.subscription_LIDAR = self.create_subscription(LaserScan, '/scan', self.get_Lidar_msg, qos_profile)
#         self.publisher = self.create_publisher(Point,'/get_object_range/distance', qos_profile)
#         self.get_logger().info("Get object range node has been started")

#     def get_Lidar_msg(self, Lidar_msg):
        
#         angleMin_rad = Lidar_msg.angle_min
#         angleMax_rad = Lidar_msg.angle_max
#         angleInc_rad = Lidar_msg.angle_increment
#         lidar_Raw_Ranges = Lidar_msg.ranges




#     def get_angle(self, msg):
#         self.angle = msg.x

#     def get_distance(self,msg):
#         self.unit_angle = msg.angle_increment # get the unit angle value of Lidar
            
#         ### get the radians in the Lidar frame
#         if self.angle >= 0:
#             radians = (self.angle/180)*np.pi
#         else:
#             radians = (self.angle/180+2)*np.pi
#         index = min(int(radians/self.unit_angle),len(msg.ranges)-1)

#         ### if not detected 
#         if self.angle == 9999.0: 
#             self.distance = 0.4 # safe distance
#         else:
#             num = 0
#             dist = 0
#             r = 2 
#             for i in range (r*2):
#                 idx = index-r+i
#                 if index < 0:
#                     idx = len(msg.ranges)+idx
#                 if idx >= len(msg.ranges):
#                     idx -= len(msg.ranges)
#                 if np.isnan(msg.ranges[idx]) or msg.ranges[idx]>=1.8:
#                     continue
#                 num += 1
#                 dist += msg.ranges[idx]
#             if num == 0:
#                 return
                
#             self.distance = dist/num

#             # self.distance = msg.ranges[index]
#             self.get_logger().info(f'Received LiDAR scan with {self.distance}')
#             # self.get_logger().info(f'i-1:Received LiDAR scan with {msg.ranges[index-1]}')
            
#         # publish msg
#         self.msg1 = Point()
#         self.msg1.x = self.angle
#         self.msg1.y = self.distance
#         self.publisher.publish(self.msg1)    


# def main():
#     rclpy.init()
#     get_object_range1 = get_object_range()
#     rclpy.spin(get_object_range1)
#     get_object_range1.destroy_node()  
#     rclpy.shutdown()

# if __name__ == '__main__':
# 	main()
