from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

def generate_launch_description():


    # Pad naar het URDF/Xacro-bestand
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'), 
        'description',
        'ackermann.urdf.xacro'
    ])

    # Converteer Xacro naar URDF (indien van toepassing)
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}

    # Pad naar de configuratie van de slam_toolbox
    slam_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'config',
        'slam_params.yaml'
    ])

    # Node voor robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Node voor slam_toolbox
    slamtoolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )


    return LaunchDescription([
        robot_state_publisher,
        slamtoolbox

    ])
