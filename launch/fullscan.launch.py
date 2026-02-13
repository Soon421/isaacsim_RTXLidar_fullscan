from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fullscan',
            executable='fullscan_node',
            name='fullscan_node',
            parameters=[{
                'input_topic': '/point_cloud',
                'output_topic': '/velodyne_points',
                'frame_id': 'velodyne',
                'num_zones': 3,
                'zone_tolerance_deg': 20.0,
                'timeout_sec': 0.5,
                'max_points': 300000,
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ])
