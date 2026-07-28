# Copyright (c) 2026 Cornel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Directories and Paths
    linorobot2_navigation_share = FindPackageShare('linorobot2_navigation')
    linorobot2_gazebo_share = FindPackageShare('linorobot2_gazebo')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    mrtsp_exploration_share = FindPackageShare('mrtsp_exploration_ros2')

    # Simulation world configuration
    world_name = LaunchConfiguration('world_name')
    gui = LaunchConfiguration('gui')
    
    # Launch configurations
    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')

    # Config Paths
    nav2_config_path = PathJoinSubstitution(
        [linorobot2_navigation_share, 'config', 'navigation.yaml']
    )
    
    rviz_config_path = PathJoinSubstitution(
        [linorobot2_navigation_share, 'rviz', 'visual_slam_explore.rviz']
    )

    # 1. Gazebo Simulation Include
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([linorobot2_gazebo_share, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world_name': world_name,
            'gui': gui
        }.items()
    )

    # 2. RTAB-Map Visual SLAM Node
    # In ROS 2 Jazzy, rtabmap is in the rtabmap_slam package
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'queue_size': 30,
            
            # SLAM settings
            'Grid/FromDepth': 'true',       # Generate 2D map from depth image
            'Reg/Force3DoF': 'true',        # Planar constraint (2D)
            'Optimizer/Slam2D': 'true',     # 2D Graph optimization
            
            # Map parameters (optimized for exploration)
            'Grid/RayTracing': 'true',       # Raytrace to clear obstacle free space
            'Grid/MaxObstacleHeight': '1.5', # Avoid mapping ceiling or high obstacles
            'Grid/MinGroundHeight': '0.04',  # Filter out floor surface noise
            'Grid/CellSize': '0.05',         # Map resolution (5cm)
            
            # Memory optimization
            'Rtabmap/DetectionRate': '2.0',  # SLAM update rate (Hz)
            'Kp/MaxFeatures': '400',
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('grid_map', '/map')
        ],
        arguments=['-d'] # Force-clear the database for a clean start
    )

    # 3. Nav2 Navigation Stack Include
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': sim,
            'params_file': nav2_config_path
        }.items()
    )

    # 4. Frontier Exploration Include (mrtsp_exploration_ros2)
    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mrtsp_exploration_share, 'launch', 'explore.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': sim
        }.items()
    )

    # 5. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': sim}]
    )

    # 6. Camera View Node (image_view)
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        parameters=[{'use_sim_time': sim}],
        remappings=[
            ('image', '/camera/color/image_raw')
        ],
        condition=IfCondition(LaunchConfiguration('camera_view'))
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            name='sim',
            default_value='true',
            description='Enable simulation time'
        ),
        DeclareLaunchArgument(
            name='world_name',
            default_value='house',
            description='Gazebo world to load'
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Start Gazebo UI client'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            name='camera_view',
            default_value='true',
            description='Launch camera image viewer window'
        ),

        # Nodes and Launches
        gazebo_launch,
        rtabmap_node,
        nav2_launch,
        exploration_launch,
        rviz_node,
        image_view_node
    ])
