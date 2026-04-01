import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    cartographer_config_dir = os.path.expanduser(
        '~/ros2_ws/src/hamerschlag_haul/config'
    )
    urdf_file = os.path.expanduser(
        '~/ros2_ws/src/hamerschlag_haul/urdf/robot.urdf'
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'serial_baudrate': 115200
        }],
        output='screen'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'rplidar_a1.lua'
        ],
        remappings=[('scan', '/scan')]
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05']
    )

    delayed_cartographer = RegisterEventHandler(
        OnProcessStart(
            target_action=rplidar_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[cartographer_node, cartographer_grid_node]
                )
            ]
        )
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file).read(),
                'use_sim_time': False,
                'publish_frequency': 100.0,
                'ignore_timestamp': True
            }]
        ),
        rplidar_node,
        delayed_cartographer,
    ])
