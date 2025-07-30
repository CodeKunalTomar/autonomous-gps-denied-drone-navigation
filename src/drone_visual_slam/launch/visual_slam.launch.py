from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Visual odometry node - IMPROVED synchronization
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'vo',
                'subscribe_rgbd': False,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'queue_size': 100,
                'sync_queue_size': 100,
                'Vis/MaxFeatures': '400',
                'Vis/MinInliers': '15'
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info')
            ]
        ),

        # SLAM Bridge node
        Node(
            package='drone_visual_slam',
            executable='slam_bridge',
            name='slam_bridge',
            output='screen'
        )
    ])
