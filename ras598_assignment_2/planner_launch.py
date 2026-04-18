#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('ras598_assignment_2')

    map_yaml_path = os.path.join(package_share, 'map.yaml')
    scout_script_path = os.path.join(package_share, 'grading_scout.py')
    rviz_config_path = os.path.join(package_share, 'planning.rviz')
    stage_launch_path = os.path.join(
        get_package_share_directory('stage_ros2'),
        'launch',
        'demo.launch.py'
    )

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': True
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    planner_node = Node(
        package='ras598_assignment_2',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    grading_scout = ExecuteProcess(
        cmd=['python3', scout_script_path],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage_launch_path),
            launch_arguments={
                'world': 'cave',
                'use_stamped_velocity': 'false'
            }.items()
        ),
        map_server,
        lifecycle_manager,
        planner_node,
        grading_scout,
        rviz_node,
    ])