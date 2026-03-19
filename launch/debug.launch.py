from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fullscan',
            executable='fullscan_node',
            name='fullscan_lidar_a',
            parameters=[{
                'input_topic': '/point_cloud',
                'output_topic': '/velodyne_points',
                'frame_id': 'velodyne',
                'num_zones': 3,
                'zone_tolerance_deg': 20.0,
                'timeout_sec': 0.5,
                'max_points': 300000,
                'target_hz': 10.0,
                'debug_mode': True,
                'use_sim_time': True,
            }],
            output='screen',
        ),
        Node(
            package='fullscan',
            executable='fullscan_node',
            name='fullscan_lidar_b',
            parameters=[{
                'input_topic': '/point_cloud_gt',
                'output_topic': '/velodyne_points_gt',
                'frame_id': 'velodyne',
                'num_zones': 3,
                'zone_tolerance_deg': 20.0,
                'timeout_sec': 0.5,
                'max_points': 500000,
                'target_hz': 10.0,
                'debug_mode': True,
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ])
