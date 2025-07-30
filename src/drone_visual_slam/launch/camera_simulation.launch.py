from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with camera world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Camera simulation node
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='camera_spawn',
            arguments=['-entity', 'test_camera', '-x', '0', '-y', '0', '-z', '1'],
            output='screen'
        ),
        
        # Test camera publisher (for initial testing)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='test_camera',
            parameters=[{
                'image_size': [640, 480],
                'time_per_frame': [1, 30],
                'camera_frame_id': 'camera_link'
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info')
            ]
        )
    ])
