from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = '/home/jinwoo/Desktop/teleop_prj/FACTR_materials/FACTR_Hardware/urdf/factr_teleop_franka.urdf'

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])

