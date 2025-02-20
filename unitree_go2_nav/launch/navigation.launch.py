from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',              #true
            choices=['true','false'],
            description='Open RVIZ for Go2 visualization'
        ),

        DeclareLaunchArgument(
            name='use_nav2_rviz',
            default_value='false',   #true
            choices=['true','false'],
            description='Open RVIZ for Nav2 visualization'
        ),

        Node(
            package='unitree_go2_nav', executable='navToPose', output='screen',
            remappings=[
                ('odom', '/utlidar/robot_odom')
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('unitree_go2_nav'),
                    'launch',
                    'mapping.launch.py'
                ])
            ),
        #     launch_arguments=[
        #         # ('use_rviz', 'false'),
        #         # ('publish_static_tf', 'false'),
        #         # ('localize_only', LaunchConfiguration('localize_only')),
        #         # ('restart_map', LaunchConfiguration('restart_map')),
        #     ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments=[
                ('params_file',
                    PathJoinSubstitution([
                        FindPackageShare('unitree_go2_nav'),
                        'config',
                        'nav2_params.yaml'
                    ])
                ),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'rviz_launch.py'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_nav2_rviz')),
        ),
    ])




