from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ros2 run orbslam3 mono ~/ORB_SLAM3_detailed_comments/Vocabulary/ORBvoc.txt ~/ws_orb3_ros2/src/ORB_SLAM3_ROS2/config/monocular/sony_8mm.yaml --ros-args --remap /camera:=/camera/live_view
    voc_file = Path(get_package_share_directory('ros2_orb_slam3'), 'orb_slam3', 'Vocabulary', 'ORBvoc.txt')
    settings_file_path = Path(get_package_share_directory('ros2_orb_slam3'), 'orb_slam3', 'config', 'Monocular')
    settings_file_name = 'sony_8mm_raw.yaml'
    # image_topic = '/camera/live_view'
    image_topic = '/camera/live_view_raw'
    # image_topic = '/cam0/image_raw'
    orb_mono = Node(
        package='ros2_orb_slam3',
        executable='mono_node_cpp',
        name='mono_node_cpp',
        output='screen',
        parameters=[
            {'voc_file': voc_file},
            {'settings_file_path': settings_file_path},
            {'settings_file_name', settings_file_name},
            {'image_topic', image_topic},
        ],
    )

    return LaunchDescription([
        orb_mono
    ])
