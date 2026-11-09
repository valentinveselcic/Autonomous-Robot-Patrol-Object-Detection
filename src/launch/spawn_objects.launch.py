from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rob_patroller')
    model_path = os.path.join(pkg_share, 'models', 'yellow_box.sdf')

    positions = [
        {'name': 'box2', 'x': '3.0', 'y': '0.0'},
        {'name': 'box3', 'x': '-5.0', 'y': '0.0'},
        {'name': 'box1', 'x': '-5.0', 'y': '0.0'}
    ]
    
    nodes = []
    for pos in positions:
        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', model_path,
                    '-name', pos['name'],
                    '-x', pos['x'], '-y', pos['y'], '-z', '0.1'
                ],
                output='screen'
            )
        )

    return LaunchDescription(nodes)
