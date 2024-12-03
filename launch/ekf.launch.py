#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, \
    TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description."""

    package_name = FindPackageShare('nubot_nav').find('nubot_nav')
    cartographer_config_dir = PathJoinSubstitution([package_name, 'config'])
    cartographer_config_basename = TextSubstitution(text='slam.lua')

    # Declare launch arguments
    use_nubot_rviz = DeclareLaunchArgument(
        'use_nubot_rviz',
        default_value='false',
        description="Launch Rviz in nubot or not."
    )
    world = DeclareLaunchArgument(
        'world',
        default_value='nubot_world.sdf',
        description="Which world to use."
    )
    publish_gazebo_tf = DeclareLaunchArgument(
        'publish_gazebo_tf',
        default_value='false',
        description="Publish TF in Gazebo or not."
    )
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=[FindPackageShare('nubot_nav').find('nubot_nav'), '/rviz/slam.rviz'],
        description="Customized Rviz configuration file."
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Use simulation clock or not."
    )
    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=[FindPackageShare('nubot_nav').find('nubot_nav'), '/config/nav2_params.yaml'],
        description="Path to Nav2 params file."
    )

    # Launch nubot simulation
    simulate_launch = IncludeLaunchDescription(
        [FindPackageShare('nubot'), '/launch/simulate.launch.xml'],
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_nubot_rviz'),
            'world': LaunchConfiguration('world'),
            'publish_tf': LaunchConfiguration('publish_gazebo_tf')
        }.items()
    )

    # Launch Nav2 stack
    nav2_bringup = IncludeLaunchDescription(
        [FindPackageShare('nav2_bringup').find('nav2_bringup'), '/launch/bringup_launch.py'],
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params')
        }.items()
    )

    # Cartographer node
    cartographer_node = Node(
        name='cartographer_node',
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename
            ],
        remappings=[('/imu/data', 'imu')]
    )

    # Cartographer occupancy grid node
    cartographer_occupancy_grid_node = Node(
        name='cartographer_occupancy_grid_node',
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'-resolution': '0.05'},
            {'-publish_period_sec': '1.0'}
        ]
    )

    # EKF node
    footprint_to_odom_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='footprint_to_odom_ekf',
        output='screen',
        parameters=[
            {'base_link_frame': 'base_footprint'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            [FindPackageShare('nubot_nav').find('nubot_nav'), '/config/footprint_to_odom_ekf.yaml'],
        ],
        remappings=[('odometry/filtered', 'odom')],
    )

    # RViz node (conditionally launched)
    rviz2_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=UnlessCondition(LaunchConfiguration('use_nubot_rviz'))
    )

    return LaunchDescription([
        # use_nubot_rviz,
        # world,
        # publish_gazebo_tf,
        # rviz_config,
        use_sim_time,
        # nav2_params,
        # simulate_launch,
        # nav2_bringup,
        # cartographer_node,
        # cartographer_occupancy_grid_node,
        footprint_to_odom_ekf,
        # rviz2_node
    ])
