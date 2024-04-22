from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 

def generate_launch_description():

    v412_camera_node =  Node(
        package = 'v4l2_camera',
        executable = 'v4l2_camera_node',
        name = 'v4l2_camera_node',
        parameters = ['./v412_camera.yaml']
    )

    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('turtlebot3_bringup'), 'launch'),
         '/robot.launch.py'])
    )

    turtlebot3_teleop_node = Node(
        package = 'turtlebot3_teleop',
        executable = 'teleop_keyboard',
        name = 'teleop_keyboard_node'
    )
    
    print_odom_node = Node(
        package = 'lab4',
        executable = 'print_fixed_odom',
        name = 'print_fixed_odometry'        
    )

    goToGoal_node = Node(
        package = 'lab4',
        executable = 'goToGoal',
        name = 'goToGoal'
    )

    get_object_range_node = Node(
        package = 'lab4',
        executable = 'get_object_range',
        name = 'get_object_range'
    )

    return LaunchDescription([
        # v412_camera_node,
        # print_odom_node,
        turtlebot3_bringup,
        get_object_range_node,
        goToGoal_node
    ])
