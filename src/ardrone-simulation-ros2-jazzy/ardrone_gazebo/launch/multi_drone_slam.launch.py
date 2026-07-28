import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_ardrone_gazebo = get_package_share_directory('ardrone_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_mrtsp_exploration = get_package_share_directory('mrtsp_exploration_ros2')
    
    # Configure Gazebo environment variables
    models_path = os.path.join(pkg_ardrone_gazebo, 'models')
    sep = os.pathsep
    existing_paths = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (models_path + sep + existing_paths) if existing_paths else models_path

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world_name')

    # ==================== SHARED INFRASTRUCTURE ====================

    # 1. Gazebo Simulation (single instance, shared)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', pkg_ardrone_gazebo, '/worlds/', world]}.items()
    )

    # TF Relay Drone 1
    tf_relay_drone1 = Node(
        package='ardrone_gazebo',
        executable='tf_relay.py',
        name='tf_relay',
        output='screen'
    )

    # TF Relay Drone 2
    tf_relay_drone2 = Node(
        package='ardrone_gazebo',
        executable='tf_relay.py',
        name='tf_relay',
        output='screen'
    )

    # ==================== STATIC SHARED ====================

    sdf_file_1 = os.path.join(pkg_ardrone_gazebo, 'models', 'ardrone_gazebo', 'ardrone_gazebo.sdf')
    urdf_file_1 = os.path.join(pkg_ardrone_gazebo, 'urdf', 'ardrone_drone1.urdf.xacro')
    nav2_config_1 = os.path.join(pkg_ardrone_gazebo, 'config', 'nav2_drone1.yaml')
    mrtsp_config_1 = os.path.join(pkg_ardrone_gazebo, 'config', 'mrtsp_drone1.yaml')

    # Spawn Drone 1
    spawn_drone1 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_drone1',
        arguments=['-name', 'ardrone_gazebo', '-file', sdf_file_1,
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    # Bridge Drone 1
    gz_bridge_drone1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_drone1',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/ardrone_gazebo/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/model/ardrone_gazebo/link/base_link/sensor/sensor_imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/drone1/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/drone1/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/drone1/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/drone1/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/model/ardrone_gazebo/odometry', '/drone1/odom_raw'),
            ('/model/ardrone_gazebo/link/base_link/sensor/sensor_imu/imu', '/drone1/ardrone/imu'),
        ],
        output='screen'
    )

    # Drone 1 Nodes (under /drone1 namespace)
    drone1_group = GroupAction([
        PushRosNamespace('drone1'),
        
        Node(
            package='ardrone_gazebo',
            executable='ardrone_driver.py',
            name='ardrone_driver',
            output='screen',
            parameters=[{'model_name': 'ardrone_gazebo'}],
            remappings=[('gz_odom', 'odom_raw')]
        ),
        Node(
            package='ardrone_gazebo',
            executable='odom_tf_publisher.py',
            name='odom_tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'drone1/odom',
                'base_frame': 'drone1/base_link'
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_1]),
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='ardrone_gazebo',
            executable='auto_takeoff.py',
            name='auto_takeoff',
            output='screen'
        ),
    ])

    # RTAB-Map Drone 1
    rtabmap_drone1 = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='drone1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'frame_id': 'drone1/base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'drone1/odom',
            'publish_tf': True,
            'approx_sync': True,
            'queue_size': 30,
            'Grid/FromDepth': 'true',
            'Reg/Force3DoF': 'true',
            'Optimizer/Slam2D': 'true',
            'Grid/RayTracing': 'true',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinGroundHeight': '0.04',
            'Grid/CellSize': '0.05',
            'Rtabmap/DetectionRate': '2.0',
            'database_path': '~/.ros/rtabmap_drone1.db',
        }],
        remappings=[
            ('rgb/image', '/drone1/camera/image'),
            ('depth/image', '/drone1/camera/depth_image'),
            ('rgb/camera_info', '/drone1/camera/camera_info'),
            ('grid_map', '/drone1/map')
        ],
        arguments=['-d']
    )

    # Nav2 Drone 1
    nav2_drone1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config_1,
            'namespace': 'drone1',
            'use_collision_monitor': 'False'
        }.items()
    )

    # MRTSP Drone 1
    mrtsp_drone1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mrtsp_exploration, 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': mrtsp_config_1,
            'namespace': 'drone1'
        }.items()
    )

    # ==================== DRONE 2 ====================

    sdf_file_2 = os.path.join(pkg_ardrone_gazebo, 'models', 'drone2', 'drone2.sdf')
    urdf_file_2 = os.path.join(pkg_ardrone_gazebo, 'urdf', 'ardrone_drone2.urdf.xacro')
    nav2_config_2 = os.path.join(pkg_ardrone_gazebo, 'config', 'nav2_drone2.yaml')
    mrtsp_config_2 = os.path.join(pkg_ardrone_gazebo, 'config', 'mrtsp_drone2.yaml')

    # Spawn Drone 2 (offset position)
    spawn_drone2 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_drone2',
        arguments=['-name', 'drone2', '-file', sdf_file_2,
                   '-x', '0.0', '-y', '2.0', '-z', '0.5'],
        output='screen'
    )

    # Bridge Drone 2
    gz_bridge_drone2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_drone2',
        arguments=[
            '/model/drone2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/drone2/link/base_link/sensor/sensor_imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/drone2/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/drone2/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/drone2/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/drone2/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/model/drone2/odometry', '/drone2/odom_raw'),
            ('/model/drone2/link/base_link/sensor/sensor_imu/imu', '/drone2/ardrone/imu'),
        ],
        output='screen'
    )

    # Drone 2 Nodes (under /drone2 namespace)
    drone2_group = GroupAction([
        PushRosNamespace('drone2'),
        
        Node(
            package='ardrone_gazebo',
            executable='ardrone_driver.py',
            name='ardrone_driver',
            output='screen',
            parameters=[{'model_name': 'drone2'}],
            remappings=[('gz_odom', 'odom_raw')]
        ),
        Node(
            package='ardrone_gazebo',
            executable='odom_tf_publisher.py',
            name='odom_tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'drone2/odom',
                'base_frame': 'drone2/base_link'
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_2]),
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='ardrone_gazebo',
            executable='auto_takeoff.py',
            name='auto_takeoff',
            output='screen'
        ),
    ])

    # RTAB-Map Drone 2
    rtabmap_drone2 = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='drone2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'frame_id': 'drone2/base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'drone2/odom',
            'publish_tf': True,
            'approx_sync': True,
            'queue_size': 30,
            'Grid/FromDepth': 'true',
            'Reg/Force3DoF': 'true',
            'Optimizer/Slam2D': 'true',
            'Grid/RayTracing': 'true',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinGroundHeight': '0.04',
            'Grid/CellSize': '0.05',
            'Rtabmap/DetectionRate': '2.0',
            'database_path': '~/.ros/rtabmap_drone2.db',
        }],
        remappings=[
            ('rgb/image', '/drone2/camera/image'),
            ('depth/image', '/drone2/camera/depth_image'),
            ('rgb/camera_info', '/drone2/camera/camera_info'),
            ('grid_map', '/drone2/map')
        ],
        arguments=['-d']
    )

    # Nav2 Drone 2
    nav2_drone2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config_2,
            'namespace': 'drone2',
            'use_collision_monitor': 'False'
        }.items()
    )

    # MRTSP Drone 2
    mrtsp_drone2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mrtsp_exploration, 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': mrtsp_config_2,
            'namespace': 'drone2'
        }.items()
    )

    # Initial static TF publishers (map -> odom) so Nav2 nodes can activate before RTAB-Map computes first pose
    static_tf_drone1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_drone1_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'drone1/odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    static_tf_drone2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_drone2_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'drone2/odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Map Merger (combines /drone1/map + /drone2/map -> /map)
    map_merger_node = Node(
        package='ardrone_gazebo',
        executable='map_merger.py',
        name='map_merger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map1_topic': '/drone1/map',
            'map2_topic': '/drone2/map',
            'merged_map_topic': '/map'
        }]
    )

    # RViz2
    rviz_config_file = os.path.join(pkg_ardrone_gazebo, 'rviz', 'visual_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Camera View Drone 1
    camera_view_drone1 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_view_drone1',
        arguments=['/drone1/camera/image']
    )

    # Camera View Drone 2
    camera_view_drone2 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_view_drone2',
        arguments=['/drone2/camera/image']
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='tugbot_depot.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Shared
        gazebo,
        # Drone 1
        spawn_drone1,
        gz_bridge_drone1,
        drone1_group,
        GroupAction([
            PushRosNamespace('drone1'),
            tf_relay_drone1,
            nav2_drone1,
        ]),
        rtabmap_drone1,
        mrtsp_drone1,
        # Drone 2
        spawn_drone2,
        gz_bridge_drone2,
        drone2_group,
        GroupAction([
            PushRosNamespace('drone2'),
            tf_relay_drone2,
            nav2_drone2,
        ]),
        rtabmap_drone2,
        mrtsp_drone2,
        # Shared nodes
        static_tf_drone1,
        static_tf_drone2,
        map_merger_node,
        rviz_node,
        camera_view_drone1,
        camera_view_drone2,
    ])
