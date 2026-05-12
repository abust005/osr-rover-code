import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': '-r osr_world.sdf' # -r starts the sim immediately; empty.sdf has basic plugins
        }.items(),
)

    osr_urdf_path = os.path.join(
        get_package_share_directory('osr_gazebo'))

    xacro_file = os.path.join(osr_urdf_path,
                              'urdf',
                              'osr.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    controller_spawn = Node(
        package='osr_gazebo',
        executable='osr_controller',
        output='screen'
    )
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'rover'],
                        output='screen')

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen'
    )
    delayed_joint_controller = TimerAction(
        period=10.0,
        actions=[load_joint_state_controller]
    )

    # wheel_velocity_controller
    rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'wheel_controller', '--controller-manager-timeout', '60'],
        output='screen'
    )
    delayed_wheel_controller = TimerAction(
        period=10.0,
        actions=[rover_wheel_controller]
    )

    # servo_controller
    servo_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'servo_controller', '--controller-manager-timeout', '60'],
        output='screen'
    )
    delay_servo_controller = TimerAction(
        period=10.0,
        actions=[servo_controller]
    )

    return LaunchDescription([
        lidar_bridge,
    	controller_spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    delayed_joint_controller,
                    delayed_wheel_controller,
                    delay_servo_controller,
                ],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
