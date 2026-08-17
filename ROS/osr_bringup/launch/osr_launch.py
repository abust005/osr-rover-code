import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    node_env = os.environ.copy()

    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'roboclaw_params.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params.yaml'
    )
    rc_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'rc_params.yaml'
    )
    crsf_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'crsf_params.yaml'
    )

    pca_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'pca9685_params.yaml'
    )

    ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package='osr_control',
            executable='roboclaw_wrapper',
            name='roboclaw_wrapper',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[roboclaw_params],
            arguments=['--ros-args', '--log-file-name', 'roboclaw_wrapper']
        )
    )

    ld.add_action(
        Node(
            package='osr_control',
            executable='crsf',
            name='crsf_rx',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[crsf_params],
            arguments=['--ros-args', '--log-file-name', 'crsf_rx']
        )
    )

    ld.add_action(
        Node(
            package='osr_control',
            executable='rc_joy',
            name='rc_joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[rc_params],
            arguments=['--ros-args', '--log-file-name', 'rc_joy']

        )
    )
    ld.add_action(
        DeclareLaunchArgument('enable_odometry', default_value='false')
    )

    pca_9685_interface_node = Node(
        package='osr_control',
        executable='pca9685_interface',
        name='pca9685_interface',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[pca_params],
        arguments=['--ros-args', '--log-file-name', 'pca9685_interface']

        
    )

    servo_control_node = Node(
        package='osr_control',
        executable='servo_control',
        name='servo_wrapper',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[{'centered_pulse_widths': [150, 152, 153, 150]}],
        arguments=['--ros-args', '--log-file-name', 'servo_control']

    )

    ld.add_action(pca_9685_interface_node)
    ld.add_action(servo_control_node)


    ld.add_action(
        DeclareLaunchArgument('enable_odometry', default_value='false')
    )
    ld.add_action(
        DeclareLaunchArgument('publish_transform', default_value='false')
    )
    ld.add_action(
        Node(
            package='osr_control',
            executable='rover',
            name='rover',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[osr_params,
                        {'enable_odometry': LaunchConfiguration('enable_odometry'),
                         'publish_transform': LaunchConfiguration('publish_transform')}],
        arguments=['--ros-args', '--log-file-name', 'rover']

        )
    )
    ld.add_action(
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                # {"scale_linear.x": 0.4},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                {"scale_linear.x": 0.6},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                # {"axis_linear.x": 4},
                {"axis_linear.x": 2},
                # {"axis_angular.yaw": 0},  # which joystick axis to use for driving
                {"axis_angular.yaw": 1},  # which joystick axis to use for driving
                # {"scale_angular.yaw": 1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"axis_angular.pitch": 0},  # axis to use for in-place rotation
                {"scale_angular.yaw": -1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular.pitch": 0.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular_turbo.yaw": 3.95},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius
                {"scale_linear_turbo.x": 1.78},  # scale to apply to linear speed, in m/s
                {"enable_button": 4},  # which button to press to enable movement
                # {"enable_button": 0},  # which button to press to enable movement
                # {"enable_turbo_button": 5}  # -1 to disable turbo
                {"enable_turbo_button": 8}  # -1 to disable turbo
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel_intuitive')
            ],
        arguments=['--ros-args', '--log-file-name', 'teleop_twist_joy']

        )
    )
    ld.add_action(
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                {"autorepeat_rate": 5.0},
                {"device_id": 0},  # This might be different on your computer. Run `ls -l /dev/input/event*`. If you have event1, put 1.
            ]        
        )
    )
    ld.add_action(
        Node(
            package='osr_control',
            executable='ina260',
            name='ina260_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"publish_rate": 1.0},
                {"sensor_address": "0x41"},
            ],
            arguments=['--ros-args', '--log-file-name', 'ina260']
        
        )
    )

    ld.add_action(
        Node(
            package='osr_control',
            executable='status_light',
            name='status_light',
            output='screen',
            emulate_tty=True,
            respawn=True,
            arguments=['--ros-args', '--log-file-name', 'status_light']
         )
     )

    ld.add_action(
        Node(
            package='osr_control',
            executable='chassis_fan',
            name='chassis_fan',
            output='screen',
            emulate_tty=True,
            respawn=True,
            arguments=['--ros-args', '--log-file-name', 'chassis_fan']

        )
    )


    # ld.add_action(
    #     Node(
    #         package='osr_control',
    #         executable='joy_extras',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[
    #             {"duty_button_index": 1}  # which button toggles duty mode on/off
    #         ]
    #     )
    # )

    return ld
