import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    cartographer_config_dir = os.path.expanduser(
        '~/ros2_ws/src/hamerschlag_haul/config'
    )
    urdf_file = os.path.expanduser(
        '~/ros2_ws/src/hamerschlag_haul/urdf/robot.urdf'
    )
    
    # Define RPLidar node separately so we can delay it
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Robot state publisher -- broadcasts base_link -> laser_frame from URDF
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
        
        # RPLiDAR node with brief delay for serial port initialization
        TimerAction(
            period=3.0,
            actions=[rplidar_node]
        ),
        
        # Cartographer SLAM node
        Node(
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
        ),
        
        # Cartographer occupancy grid node (publishes /map)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05']
        ),
    ])
