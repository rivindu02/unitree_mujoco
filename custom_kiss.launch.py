from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kiss_icp',
            executable='kiss_icp_node',
            name='kiss_icp_node',
            output='screen',
            remappings=[
                ('pointcloud_topic', '/lidar_points'),
            ],
            parameters=[{
                'use_sim_time': True,             # FORCES SYNC WITH MUJOCO
                'publish_odom_tf': True,
                'publish_debug_clouds': True,
                'data.deskew': False,
                'data.max_range': 15.0,
                'mapping.voxel_size': 0.1,
                'adaptive_threshold.min_motion_th': 0.1,
                'registration.max_num_iterations': 1000,
                'registration.convergence_criterion': 0.0001
            }]
        )
    ])
