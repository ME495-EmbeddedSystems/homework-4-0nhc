#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""Launch file for manual exploration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
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
        description='Launch Rviz in nubot or not.'
    )
    world = DeclareLaunchArgument(
        'world',
        default_value='nubot_simple.sdf',
        description='Which world to use.'
    )
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=[FindPackageShare('nubot_nav').find('nubot_nav'),
                       '/rviz/slam.rviz'],
        description='Customized Rviz configuration file.'
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock or not.'
    )
    use_jsp = DeclareLaunchArgument(
        'use_jsp',
        default_value='false',
        description='Specify the Joint State Publisher to use.'
    )
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Use Rviz in nubot or not.'
    )
    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=[FindPackageShare('nubot_nav').find('nubot_nav'),
                       '/config/nav2_params.yaml'],
        description='Path to Nav2 params file.'
    )

    gazebo_world_launch = IncludeLaunchDescription(
        [FindPackageShare('nubot_nav').find('nubot_nav'),
         '/launch/nubot_world.launch.xml'],
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    nubot_rviz_launch = IncludeLaunchDescription(
        [FindPackageShare('nubot').find('nubot'),
         '/launch/nubot_rviz.launch.py'],
        launch_arguments={
            'use_jsp': LaunchConfiguration('use_jsp'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-x', '15', '-z', '0.3'],
        output='screen'
        )

    # Launch Nav2 stack
    nav2_bringup = IncludeLaunchDescription(
        [FindPackageShare('nav2_bringup').find('nav2_bringup'),
         '/launch/bringup_launch.py'],
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

    # RViz node (conditionally launched)
    rviz2_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=UnlessCondition(LaunchConfiguration('use_nubot_rviz'))
    )

    return LaunchDescription([
        use_nubot_rviz,
        world,
        rviz_config,
        use_sim_time,
        use_jsp,
        use_rviz,
        nav2_params,
        gazebo_world_launch,
        nubot_rviz_launch,
        spawn_robot,
        nav2_bringup,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz2_node
    ])
