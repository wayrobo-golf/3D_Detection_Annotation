import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('automatic_annotation')
    params_file = os.path.join(pkg_dir, 'config', 'default.yaml')
    
    if not os.path.exists(params_file):
        print(f"警告：参数文件不存在 {params_file}")
    
    return LaunchDescription([
        Node(
            package='automatic_annotation',
            executable='automatic_annotation_node',
            name='automatic_annotation_node',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0
        )
    ])