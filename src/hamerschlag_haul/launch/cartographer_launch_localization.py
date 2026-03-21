import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cartographer_config_dir = os.path.expanduser(
        '~/ros2_ws/src/hamerschlag_haul/config'
    )

    urdf_file = os.path.expanduser(
        '~/ros2_ws/src/hamerschlag_haul/urdf/robot.urdf'
    )

    map_file = os.path.expanduser('~/map.yaml')

    return LaunchDescription([

        # Robot state publisher
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

        # RPLiDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
            }],
            output='screen'
        ),

        # Cartographer in localization mode
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'rplidar_a1_localization.lua',
                '-load_state_filename', os.path.expanduser('~/map.pbstream')
            ],
            remappings=[('scan', '/scan')]
        ),

        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05']
        ),

    ])