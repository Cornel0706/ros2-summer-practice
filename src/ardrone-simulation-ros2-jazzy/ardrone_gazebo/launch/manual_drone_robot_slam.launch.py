import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_ardrone_gazebo = get_package_share_directory('ardrone_gazebo')
    pkg_linorobot2_description = get_package_share_directory('linorobot2_description')
    
    models_path = os.path.join(pkg_ardrone_gazebo, 'models')
    sep = os.pathsep
    existing_paths = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (models_path + sep + existing_paths) if existing_paths else models_path
    os.environ['GZ_VERBOSE'] = '0'
    os.environ['LINOROBOT2_BASE'] = '2wd'

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world_name')

    # Shared Gazebo
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
        namespace='drone1',
        output='screen'
    )
    
    # TF Relay Lino
    tf_relay_lino = Node(
        package='ardrone_gazebo',
        executable='tf_relay.py',
        name='tf_relay',
        namespace='lino',
        output='screen'
    )

    # ==================== DRONE 1 ====================
    sdf_file_1 = os.path.join(pkg_ardrone_gazebo, 'models', 'ardrone_gazebo', 'ardrone_gazebo.sdf')
    urdf_file_1 = os.path.join(pkg_ardrone_gazebo, 'urdf', 'ardrone_drone1.urdf.xacro')

    spawn_drone1 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_drone1',
        arguments=['-name', 'ardrone_gazebo', '-file', sdf_file_1, '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

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

    drone1_group = GroupAction([
        PushRosNamespace('drone1'),
        Node(package='ardrone_gazebo', executable='ardrone_driver.py', name='ardrone_driver', parameters=[{'model_name': 'ardrone_gazebo'}], remappings=[('gz_odom', 'odom_raw')]),
        Node(package='ardrone_gazebo', executable='odom_tf_publisher.py', name='odom_tf_publisher', parameters=[{'use_sim_time': use_sim_time, 'odom_frame': 'drone1/odom', 'base_frame': 'drone1/base_link'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': Command(['xacro ', urdf_file_1]), 'use_sim_time': use_sim_time}]),
        Node(package='ardrone_gazebo', executable='auto_takeoff.py', name='auto_takeoff')
    ])

    rtabmap_drone1 = Node(
        package='rtabmap_slam', executable='rtabmap', namespace='drone1',
        parameters=[{
            'use_sim_time': use_sim_time, 'subscribe_depth': True, 'subscribe_rgb': True, 'subscribe_scan': False,
            'frame_id': 'drone1/base_link', 'map_frame_id': 'map', 'odom_frame_id': 'drone1/odom', 'publish_tf': False,
            'approx_sync': True, 'queue_size': 30, 'Grid/FromDepth': 'true', 'Reg/Force3DoF': 'true', 'Optimizer/Slam2D': 'true',
            'Grid/RayTracing': 'true', 'Grid/MaxObstacleHeight': '2.0', 'Grid/MinGroundHeight': '0.04', 'Grid/CellSize': '0.05',
            'Rtabmap/DetectionRate': '2.0', 'database_path': '',
        }],
        remappings=[('rgb/image', '/drone1/camera/image'), ('depth/image', '/drone1/camera/depth_image'), ('rgb/camera_info', '/drone1/camera/camera_info'), ('grid_map', '/drone1/map')],
        arguments=['-d']
    )

    # ==================== LINOROBOT ====================
    urdf_file_lino = os.path.join(pkg_linorobot2_description, 'urdf', 'robots', '2wd.urdf.xacro')

    spawn_lino = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_lino',
                arguments=['-name', 'lino', '-string', Command(['xacro ', urdf_file_lino]), '-x', '0.0', '-y', '2.0', '-z', '0.1'],
                output='screen'
            )
        ]
    )

    gz_bridge_lino = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_lino',
        arguments=[
            '/lino/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/lino/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/lino/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/lino/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/lino/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lino/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lino/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lino/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lino/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/model/lino/odometry', '/lino/odom/unfiltered'),
            ('/model/lino/imu/data', '/lino/imu/data'),
            ('/model/lino/joint_states', '/lino/joint_states'),
            ('/lino/camera/camera_info', '/lino/camera/color/camera_info'),
            ('/lino/camera/image', '/lino/camera/color/image_raw'),
            ('/lino/camera/depth_image', '/lino/camera/depth/image_rect_raw'),
            ('/lino/camera/points', '/lino/camera/depth/color/points'),
        ],
        output='screen'
    )

    lino_group = GroupAction([
        PushRosNamespace('lino'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': Command(['xacro ', urdf_file_lino]), 'use_sim_time': use_sim_time, 'frame_prefix': 'lino/'}]),
        Node(package='ardrone_gazebo', executable='odom_tf_publisher.py', name='odom_tf_publisher', parameters=[{'use_sim_time': use_sim_time, 'odom_frame': 'odom', 'base_frame': 'lino/base_footprint'}], remappings=[('odom_raw', 'odom/unfiltered')])
    ])

    rtabmap_lino = Node(
        package='rtabmap_slam', executable='rtabmap', namespace='lino',
        parameters=[{
            'use_sim_time': use_sim_time, 'subscribe_depth': True, 'subscribe_rgb': True, 'subscribe_scan': False,
            'frame_id': 'lino/base_footprint', 'map_frame_id': 'map', 'odom_frame_id': 'odom', 'publish_tf': False,
            'approx_sync': True, 'queue_size': 30, 'Grid/FromDepth': 'true', 'Reg/Force3DoF': 'true', 'Optimizer/Slam2D': 'true',
            'Grid/RayTracing': 'true', 'Grid/RangeMax': '5.0', 'Grid/MaxObstacleHeight': '2.0', 'Grid/MinGroundHeight': '0.04', 'Grid/CellSize': '0.05',
            'Rtabmap/DetectionRate': '2.0', 'database_path': '',
        }],
        remappings=[('rgb/image', '/lino/camera/color/image_raw'), ('depth/image', '/lino/camera/depth/image_rect_raw'), ('rgb/camera_info', '/lino/camera/color/camera_info'), ('grid_map', '/lino/map')],
        arguments=['-d']
    )

    # Shared TFs and Map Merger
    static_tf_drone1 = Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_drone1_odom', arguments=['0', '0', '0', '0', '0', '0', 'map', 'drone1/odom'], parameters=[{'use_sim_time': use_sim_time}])
    static_tf_lino = Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_lino_odom', arguments=['0', '2.0', '0', '0', '0', '0', 'map', 'odom'], parameters=[{'use_sim_time': use_sim_time}])

    map_merger_node = Node(
        package='ardrone_gazebo', executable='map_merger.py', name='map_merger',
        parameters=[{'use_sim_time': use_sim_time, 'map1_topic': '/drone1/map', 'map2_topic': '/lino/map', 'merged_map_topic': '/map'}]
    )

    rviz_config_file = os.path.join(pkg_ardrone_gazebo, 'rviz', 'visual_slam.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config_file], condition=IfCondition(rviz), parameters=[{'use_sim_time': use_sim_time}])

    teleop_drone1 = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_drone1',
        prefix='gnome-terminal --',
        remappings=[('cmd_vel', '/drone1/cmd_vel')],
        output='screen'
    )

    teleop_lino = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_lino',
        prefix='gnome-terminal --',
        remappings=[('cmd_vel', '/lino/cmd_vel')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='tugbot_depot.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo,
        tf_relay_drone1,
        tf_relay_lino,
        spawn_drone1,
        spawn_lino,
        gz_bridge_drone1,
        gz_bridge_lino,
        drone1_group,
        lino_group,
        rtabmap_drone1,
        rtabmap_lino,
        static_tf_drone1,
        static_tf_lino,
        map_merger_node,
        rviz_node,
        teleop_drone1,
        teleop_lino
    ])
