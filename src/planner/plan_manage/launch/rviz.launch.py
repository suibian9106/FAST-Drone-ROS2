from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录路径
    ego_planner_dir = get_package_share_directory('ego_planner')
    rviz_config = os.path.join(ego_planner_dir, 'launch', 'default.rviz')
    
    # 创建 RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_node
    ])