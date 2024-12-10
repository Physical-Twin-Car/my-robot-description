from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

def generate_launch_description():

    use_sim_time = False


    # Pad naar de configuratie van de slam_toolbox
    slam_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'config',
        'mapper_params_online_async.yaml'
    ])


    # Node voor slam_toolbox
    slamtoolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path, {'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        slamtoolbox

    ])
