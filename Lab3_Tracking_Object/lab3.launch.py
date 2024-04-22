from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 

def generate_launch_description():

    v412_camera_node =  Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        parameters=['./v412_camera.yaml']
    )

    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('turtlebot3_bringup'), 'launch'),
         '/robot.launch.py'])
    )

    detect_object = Node(
        package='rao',
        executable='detect_object',
        name='detect_object'
    )

    get_object_range = Node(
        package='rao',
        executable='get_object_range',
        name='get_object_range'
    )

    chase_object = Node(
        package='rao',
        executable='chase_object',
        name='chase_object'
    )
    return LaunchDescription([
        v412_camera_node,
        turtlebot3_bringup,
        detect_object,
        get_object_range,
        chase_object
    ])
