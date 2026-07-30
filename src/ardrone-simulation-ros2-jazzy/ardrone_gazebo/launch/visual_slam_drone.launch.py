import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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

    # Suppress verbose Gazebo wrench/entity debug output (cosmetic only)
    os.environ['GZ_VERBOSE'] = '0'

    sdf_file = os.path.join(pkg_ardrone_gazebo, 'models', 'ardrone_gazebo', 'ardrone_gazebo.sdf')
    urdf_file = os.path.join(pkg_ardrone_gazebo, 'urdf', 'ardrone.urdf.xacro')

    world = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    nav2_config_path = PathJoinSubstitution([pkg_ardrone_gazebo, 'config', 'navigation.yaml'])

    # 1. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', pkg_ardrone_gazebo, '/worlds/', world]}.items()
    )

    # 2. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ardrone',
        arguments=['-name', 'ardrone_gazebo', '-file', sdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    # 3. Gazebo ROS Bridge (updated for /drone1/camera topics)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge',
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
            ('/model/ardrone_gazebo/odometry', '/odom_raw'),
            ('/model/ardrone_gazebo/link/base_link/sensor/sensor_imu/imu', '/ardrone/imu'),
        ],
        output='screen'
    )

    # 4. AR.Drone Flight Driver
    driver_node = Node(
        package='ardrone_gazebo',
        executable='ardrone_driver.py',
        name='ardrone_driver',
        output='screen',
        parameters=[{'model_name': 'ardrone_gazebo'}],
        remappings=[
            ('gz_odom', '/odom_raw')
        ]
    )

    # 5. Odom TF Publisher
    odom_tf_node = Node(
        package='ardrone_gazebo',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('odom_raw', '/odom_raw'),
            ('odom', '/odom')
        ]
    )

    # 6. Robot State Publisher (Static TFs from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )

    # 7. Auto Takeoff after 8 seconds (Reliable Node)
    auto_takeoff = Node(
        package='ardrone_gazebo',
        executable='auto_takeoff.py',
        name='auto_takeoff',
        output='screen',
        remappings=[
            ('odom', '/odom'),
            ('ardrone/takeoff', '/ardrone/takeoff')
        ]
    )

    # 7.5 Static TF map -> odom (Required for Nav2 initialization before RTAB-Map publishes it)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 8. RTAB-Map Visual SLAM
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
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
        }],
        remappings=[
            ('rgb/image', '/drone1/camera/image'),
            ('depth/image', '/drone1/camera/depth_image'),
            ('rgb/camera_info', '/drone1/camera/camera_info'),
            ('grid_map', '/map')
        ],
        arguments=['-d']
    )

    # 9. Nav2 Navigation Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config_path,
            'use_collision_monitor': 'False'
        }.items()
    )

    # 10. MRTSP Frontier Exploration
    mrtsp_config_path = PathJoinSubstitution([pkg_ardrone_gazebo, 'config', 'mrtsp_params.yaml'])
    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mrtsp_exploration, 'launch', 'explore.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': mrtsp_config_path}.items()
    )

    # 11. RViz2
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

    # 12. RQT Image View
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/drone1/camera/image'],
        condition=IfCondition(rviz)
    )

    # 13. Virtual Battery (Smart RTH)
    virtual_battery_node = Node(
        package='ardrone_gazebo',
        executable='virtual_battery.py',
        name='virtual_battery',
        output='screen',
        remappings=[
            ('odom', '/odom'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='tugbot_depot.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo,
        spawn_robot,
        gz_bridge,
        driver_node,
        odom_tf_node,
        robot_state_publisher,
        auto_takeoff,
        static_tf_node,
        rtabmap_node,
        nav2_launch,
        exploration_launch,
        rviz_node,
        rqt_image_view_node,
        virtual_battery_node
    ])
