import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ardrone_gazebo = get_package_share_directory('ardrone_gazebo')
    models_path = os.path.join(pkg_ardrone_gazebo, 'models')
    
    # Configure Gazebo environment variables
    # Set GZ_SIM_RESOURCE_PATH to include our models so Gazebo can find visual/collision meshes
    sep = os.pathsep
    existing_paths = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (models_path + sep + existing_paths) if existing_paths else models_path

    sdf_file = os.path.join(pkg_ardrone_gazebo, 'models', 'ardrone_gazebo', 'ardrone_gazebo.sdf')
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, PythonExpression

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tugbot_depot.sdf',
        description='World file to launch (e.g. tugbot_depot.sdf, rubicon.sdf, ardrone_empty.sdf)'
    )

    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (server only)'
    )

    gz_args_expr = [
        '-r',
        PythonExpression(['" -s" if "', headless, '" in ["true", "True"] else ""']),
        ' ',
        PythonExpression(['"', pkg_ardrone_gazebo, '/worlds/', world, '"'])
    ]

    # Launch Gazebo Sim with our custom world containing ApplyLinkWrench system plugin
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args_expr}.items()
    )

    # Spawn AR.Drone model in Gazebo slightly above terrain
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ardrone',
        arguments=[
            '-name', 'ardrone_gazebo',
            '-file', sdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # Bridge ROS 2 and Gazebo Sim topics
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/ardrone_gazebo/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/model/ardrone_gazebo/link/base_link/sensor/sensor_imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/ardrone_gazebo/link/base_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/ardrone_gazebo/link/base_link/sensor/down_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/model/ardrone_gazebo/link/base_link/sensor/sensor_imu/imu', '/ardrone/imu'),
            ('/model/ardrone_gazebo/link/base_link/sensor/front_camera/image', '/front_camera/image_raw'),
            ('/model/ardrone_gazebo/link/base_link/sensor/down_camera/image', '/down_camera/image_raw'),
        ],
        output='screen'
    )

    # Launch simulation control driver node
    driver_node = Node(
        package='ardrone_gazebo',
        executable='ardrone_driver.py',
        name='ardrone_driver',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        gazebo,
        spawn_robot,
        gz_bridge,
        driver_node
    ])
