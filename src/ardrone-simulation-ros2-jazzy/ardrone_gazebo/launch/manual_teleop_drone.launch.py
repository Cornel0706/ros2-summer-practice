import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_ardrone_gazebo = get_package_share_directory('ardrone_gazebo')
    
    # Configure Gazebo environment variables
    models_path = os.path.join(pkg_ardrone_gazebo, 'models')
    sep = os.pathsep
    existing_paths = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (models_path + sep + existing_paths) if existing_paths else models_path

    sdf_file = os.path.join(pkg_ardrone_gazebo, 'models', 'ardrone_gazebo', 'ardrone_gazebo.sdf')
    urdf_file = os.path.join(pkg_ardrone_gazebo, 'urdf', 'ardrone.urdf.xacro')

    world = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

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

    # 3. Gazebo ROS Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/ardrone_gazebo/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/model/ardrone_gazebo/link/base_link/sensor/sensor_imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
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
        remappings=[
            ('model/ardrone_gazebo/odometry', '/odom')
        ]
    )

    # 5. Odom TF Publisher
    odom_tf_node = Node(
        package='ardrone_gazebo',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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

    # 7. Auto Takeoff
    auto_takeoff = Node(
        package='ardrone_gazebo',
        executable='auto_takeoff.py',
        name='auto_takeoff',
        output='screen'
    )

    # 8. Visual SLAM (RTAB-Map)
    rtabmap_parameters = {
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'approx_sync': True,
        'use_sim_time': use_sim_time,
        'Grid/FromDepth': 'true',
        'Grid/RangeMax': '5.0',
        'Grid/RayTracing': 'true',
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
        'RGBD/ProximityBySpace': 'false'
    }

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('depth/image', '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom')
        ],
        arguments=['--delete_db_on_start']
    )

    # 9. RViz2
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

    # 10. RQT Image View
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/camera/image'],
        condition=IfCondition(rviz)
    )

    # 11. Teleop Twist Keyboard (deschide un terminal nou)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='gnome-terminal --'
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
        rtabmap_node,
        rviz_node,
        rqt_image_view_node,
        teleop_node
    ])
