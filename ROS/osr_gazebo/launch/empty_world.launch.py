import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    osr_gazebo_path = get_package_share_directory('osr_gazebo')
    world_name = 'osr_world.sdf'
    world_path = os.path.join(osr_gazebo_path, 'worlds', world_name)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': f'-r {world_path}' 
        }.items()
    )

    xacro_file = os.path.join(osr_gazebo_path, 'urdf', 'osr.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params.yaml',
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    joint_state_remapper = Node(
        package='osr_gazebo',
        executable='joint_state_adapter.py',
        name='joint_state_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )


    rover_node = Node(
        package='osr_control',
        executable='rover',
        name='rover',
        output='screen',
        parameters=[
            osr_params,
            {'enable_odometry': True, 'publish_transform': True, 'use_sim_time': True},
        ],
        respawn=True,
    )

    gazebo_adapter = Node(
        package='osr_gazebo',
        executable='gazebo_command_adapter.py',
        name='gazebo_command_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'rover',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.25'],
        output='screen',
    )

    # Clock bridge for sim time sync across ROS nodes
    ros_gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   ],

        output='screen',
    )

    # 4. Modern Controller Spawners (Sequential Execution)
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_servo_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['servo_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Chain spawners sequentially to prevent controller_manager switch locks
    chain_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_wheel_controller],
        )
    )

    chain_servo_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_wheel_controller,
            on_exit=[load_servo_controller],
        )
    )

    # Trigger controller sequence after spawn finishes with a short stabilization buffer
    delay_spawners_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[load_jsb],
                )
            ],
        )
    )



    return LaunchDescription([
        ros_gz_clock_bridge,
        joint_state_remapper,
        rover_node,
        gazebo_adapter,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delay_spawners_after_spawn,
        chain_wheel_controller,
        chain_servo_controller,
    ])