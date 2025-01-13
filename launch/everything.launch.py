from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # get package share directory
    rf2o_laser_odometry_share = get_package_share_directory('rf2o_laser_odometry')
    my_robot_description_share = get_package_share_directory('my_robot_description')

    return LaunchDescription([
        # RPLidar Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'angle_compensate': True
            }]
        ),

        # Laser Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rf2o_laser_odometry_share, 'launch', 'rf2o_laser_odometry.launch.py')
            )
        ),


        # Robot Description Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_robot_description_share, 'launch', 'robot.launch.py')
            )
        ),

        # Motor Control Node
        Node(
            package='motor_control_pkg',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'
        )
    ])
