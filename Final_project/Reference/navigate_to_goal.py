import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose
import numpy as np

class Navigator(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')

        self.subscription = self.create_subscription(
            Int32,
            '/sign_state',
            self.state_callback,
            10)
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribe to the goal status topic to get feedback on the goal status
        self.status_subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_reached_callback,
            10)
        
        # # Subscribe to the NavigateToPose feedback topic
        # self.feedback_subscription = self.create_subscription(
        #     NavigateToPose.Feedback,
        #     '/navigate_to_pose/_action/feedback',
        #     self.feedback_callback,
        #     10)    

        # Flag to indicate whether the initial waypoint navigation has been completed
        self.initial_waypoint_navigation_completed = False

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"

        # Flag to indicate whether the robot is currently navigating to a goal
        self.is_navigating = False

        # Timer for publishing the goal pose
        self.publish_timer = None
        self.timer = self.create_timer(0.2, self.navigate_to_waypoint)

        # navigation state
        self.state = 0

        # Current pose (We can get it from publishing clicked point)
        self.current_pose = NavigateToPose.Feedback()
        self.current_pose.current_pose.pose.position.x = 4.49 # change this
        self.current_pose.current_pose.pose.position.y = -1.056  # change this

        # List of waypoints (17 points) 0-16
        self.waypoints = [
        [0.854, 0.861, 0.0],
        [0.04,0.888,0.0],
        [0.0,0.0,0.0],
        [0.868,-0.048,0.0],
        [0.829,-1.017,0.0],
        [1.674,-0.972,0.0],
        [1.77,-0.008,0.0],
        [1.79,0.877,0.0],
        [2.731,0.855,0.0],
        [2.725,-0.181,0.0],
        [2.731,-1.017,0.0],
        [3.666,-1.067,0.0],
        [4.49,-1.056,0.0],
        [4.49,-0.15,0.0],
        [4.49,0.8,0.0],
        [3.66,0.87,0.0],
        [3.66,-0.098,0.0]
        ]
        self.current_waypoint_index = -1  #initialize

    def find_closest_waypoint(self):
        """
        Find the index of the closest waypoint to the robot's current position when it starts
        """
        if self.current_pose is None:
            return None

        min_distance = float('inf')
        closest_index = None
        for i, waypoint in enumerate(self.waypoints):
            distance = np.sqrt(
                (self.current_pose.current_pose.pose.position.x - waypoint[0]) ** 2 +
                (self.current_pose.current_pose.pose.position.y - waypoint[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index
    
    # def feedback_callback(self, msg):
    #     """
    #     Callback function to process the feedback from the NavigateToPose action.
    #     """
    #     self.current_pose.current_pose = msg.current_pose
    #     self.get_logger().info(
    #         f"Current Pose from Feedback - Position: (x: {self.current_pose.current_pose.pose.position.x}, "
    #         f"y: {self.current_pose.current_pose.pose.position.y}, z: {self.current_pose.current_pose.pose.position.z}), "
    #         f"Orientation: (x: {self.current_pose.current_pose.pose.orientation.x}, "
    #         f"y: {self.current_pose.current_pose.pose.orientation.y}, z: {self.current_pose.current_pose.pose.orientation.z}, "
    #         f"w: {self.current_pose.current_pose.pose.orientation.w})"
    #     )

    def goal_reached_callback(self, msg):
        """
        Callback function to process the goal status.
        """
        if len(msg.status_list) > 0:
            status = msg.status_list[-1].status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Waypoint reached.')
                self.is_navigating = False
                
                #update current pose
                self.current_pose.current_pose.pose.orientation.w = self.goal_pose.pose.orientation.w
                self.current_pose.current_pose.pose.orientation.x = self.goal_pose.pose.orientation.x
                self.current_pose.current_pose.pose.orientation.y = self.goal_pose.pose.orientation.y
                self.current_pose.current_pose.pose.orientation.z = self.goal_pose.pose.orientation.z
                
                # if self.publish_timer:
                #     self.publish_timer.cancel()
                #     self.publish_timer = None
                # self.navigate_to_waypoint()  # Navigate to the next waypoint
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.get_logger().info('Waypoint could not be reached.')
                self.is_navigating = False
                # if self.publish_timer:
                #     self.publish_timer.cancel()
                #     self.publish_timer = None

    # def publish_goal_pose(self, goal_pose):
    #     if not self.publish_timer:
    #         self.publish_timer = self.create_timer(0.2, lambda: self.publisher.publish(goal_pose))

    def state_callback(self, msg):
        state = msg.data

        # State logic here
        if self.is_navigating:
            # Wait until the current goal is reached or canceled before setting a new goal
            self.state = None
            return
        
        # Update state
        self.state = state
        self.get_logger().info(f'Received state: {state}')

    def navigate_to_waypoint(self):
        # We just started 
        if self.current_waypoint_index == -1:
            self.current_waypoint_index = self.find_closest_waypoint()
            self.get_logger().info(f'closest waypoint index: {self.current_waypoint_index}')
            waypoint = self.waypoints[self.current_waypoint_index]

            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.pose.position.x = waypoint[0]
            self.goal_pose.pose.position.y = waypoint[1]
            # # publish goal pose
            # self.pose_publisher.publish(goal_pose)
            # self.is_navigating = True
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index}: {waypoint}')

        # Waypoint update logic
        else:
            # point 0
            if self.current_waypoint_index == 0:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 1.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 0.0 
                elif self.state == 1:
                    # turn left by 90
                    self.current_waypoint_index = 1
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]
                elif self.state == 2:
                    # turn right by 90
                    self.current_waypoint_index = 1
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]                    
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 1
                    self.current_waypoint_index = 1
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]  

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')
            # point 1
            elif self.current_waypoint_index == 1:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 0.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 1.0 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        print(np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0))
                        print(np.abs(np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4))))
                        self.current_waypoint_index = 2
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        print(np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0))
                        print(np.abs(np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4))))
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 0.0 
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 1.0
                        

                elif self.state == 3 or self.state == 4:
                    # turn back
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                    # in this case go to 2
                        self.current_waypoint_index = 2
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]   
                    else:
                        # in this case go to 0
                        self.current_waypoint_index = 0
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]  

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!') 

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 0
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = np.sin(np.pi/4)

            # point 2
            elif self.current_waypoint_index == 2:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 6
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 
                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 1
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 0.0 
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 1.0

                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 6
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 6
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 1
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]               
                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')
            # point 3
            elif self.current_waypoint_index == 3:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = np.sin(np.pi/4) 
                elif self.state == 1:
                    # turn left by 90
                    self.current_waypoint_index = 2
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]
                elif self.state == 2:
                    # turn right by 90
                    self.current_waypoint_index = 6
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]                    
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 6
                    self.current_waypoint_index = 4
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]    

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')
            # point 4  
            elif self.current_waypoint_index == 4:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face right
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 5
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 3
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 0.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 1.0 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 5
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 5
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 3
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')
            # point 5  
            elif self.current_waypoint_index == 5:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face right
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 1 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 0.0 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 7
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 1.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 0.0 

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 4
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 5
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 7
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 4
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')
            # point 6  
            elif self.current_waypoint_index == 6:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 1.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 0.0 
                elif self.state == 1:
                    # turn left by 90
                    self.current_waypoint_index = 7
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]
                elif self.state == 2:
                    # turn right by 90
                    self.current_waypoint_index = 5
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]                    
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 6
                    self.current_waypoint_index = 2
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]    

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 7  
            elif self.current_waypoint_index == 7:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face right
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = np.sin(np.pi/4) 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 5
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 0.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 1.0 

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 8
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = np.sin(np.pi/4) 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 5
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 8
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 5
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 8  
            elif self.current_waypoint_index == 8:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face right
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 1.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 0.0 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) > np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 7
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = np.sin(np.pi/4)

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 10
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 1.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 0.0 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 5
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 7
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 10
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 9  
            elif self.current_waypoint_index == 9:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 0.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 1.0 
                elif self.state == 1:
                    # turn left by 90
                    self.current_waypoint_index = 10
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]
                elif self.state == 2:
                    # turn right by 90
                    self.current_waypoint_index = 8
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]                    
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 13
                    self.current_waypoint_index = 13
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]    

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 10  
            elif self.current_waypoint_index == 10:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face down
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 0.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 1.0 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 12
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = -np.sin(np.pi/4)

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 8
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 0.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 1.0 
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 8 or 12
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 12
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 8
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 11  
            elif self.current_waypoint_index == 11:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face down
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 12
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 10
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 12
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.current_waypoint_index = 10
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 

                elif self.state == 3 or self.state == 4:
                    # shouldn't happen but go to 10
                    # 
                    self.current_waypoint_index = 10
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 12  
            elif self.current_waypoint_index == 12:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face up
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 1 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 0.0 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) < np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 14
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 1.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 0.0 

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 10
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = -np.sin(np.pi/4) 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 14 or 10
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) > np.abs(self.current_pose.current_pose.pose.orientation.z + np.sin(np.pi/4)):
                        self.current_waypoint_index = 14
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 10
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')  
            # point 13
            elif self.current_waypoint_index == 13:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = 1.0 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = 0.0 
                elif self.state == 1:
                    # turn left by 90
                    self.current_waypoint_index = 14
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]
                elif self.state == 2:
                    # turn right by 90
                    self.current_waypoint_index = 12
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]                    
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 9
                    self.current_waypoint_index = 9
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]    

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')

            # point 14
            elif self.current_waypoint_index == 14:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face left
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = np.sin(np.pi/4)

                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) > np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 15
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = np.sin(np.pi/4)

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 12
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 1.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 0.0 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 15 or 12
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 0.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 15
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 12
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')  
            # point 15
            elif self.current_waypoint_index == 15:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    # face left
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = np.sin(np.pi/4) 
                elif self.state == 1:
                    # turn left by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 16
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = 0.0
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = 1.0 

                elif self.state == 2:
                    # turn right by 90
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) > np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 14
                        waypoint = self.waypoints[self.current_waypoint_index]

                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1] 
                    else:
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.orientation.w = np.cos(np.pi/4)
                        self.goal_pose.pose.orientation.x = 0.0
                        self.goal_pose.pose.orientation.y = 0.0
                        self.goal_pose.pose.orientation.z = np.sin(np.pi/4) 
                  
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 14 or 16
                    if np.abs(self.current_pose.current_pose.pose.orientation.z - 1.0) < np.abs(self.current_pose.current_pose.pose.orientation.z - np.sin(np.pi/4)):
                        self.current_waypoint_index = 14
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]
                    else:
                        self.current_waypoint_index = 16
                        waypoint = self.waypoints[self.current_waypoint_index]


                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pose.pose.position.x = waypoint[0]
                        self.goal_pose.pose.position.y = waypoint[1]

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')  
                    
            # point 16
            elif self.current_waypoint_index == 16:
                if self.state == 0:
                    # empty wall, just turn
                    # cmd_msg = Twist()
                    # cmd_msg.linear.x = 0.0
                    # cmd_msg.angular.z = 0.5
                    # self.vel_publisher.publish(cmd_msg)
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.orientation.w = np.cos(np.pi/4) 
                    self.goal_pose.pose.orientation.x = 0.0
                    self.goal_pose.pose.orientation.y = 0.0
                    self.goal_pose.pose.orientation.z = -np.sin(np.pi/4)

                elif self.state == 1:
                    # turn left by 90
                    self.current_waypoint_index = 13
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]
                elif self.state == 2:
                    # turn right by 90
                    self.current_waypoint_index = 9
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]                    
                elif self.state == 3 or self.state == 4:
                    # turn back
                    # in this case go to 15
                    self.current_waypoint_index = 15
                    waypoint = self.waypoints[self.current_waypoint_index]

                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = waypoint[0]
                    self.goal_pose.pose.position.y = waypoint[1]    

                elif self.state == 5:
                    #goal reached 
                    self.get_logger().info(f'Goal reached!')  

        # publish goal pose
        self.pose_publisher.publish(self.goal_pose)
        self.is_navigating = True
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index}!')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    navigator = Navigator()
    
    # Spin the node so the callback function is called.
    rclpy.spin(navigator)

    # Destroy the node explicitly
    navigator.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
