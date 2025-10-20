import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取vins功能包的共享目录路径
    vins_para_path=os.path.join(get_package_share_directory('vins'), 'config','fast_drone_250.launch.py')
    
    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_fusion',
        output='screen',
        parameters=[vins_para_path,]
    )
    
    # 被注释掉的loop_fusion节点（如需使用，取消注释）
    # loop_fusion_node = Node(
    #     package='loop_fusion',
    #     executable='loop_fusion_node',
    #     name='loop_fusion',
    #     output='screen',
    #     parameters=[config_path]
    # )
    
    return LaunchDescription([
        vins_node,
        # loop_fusion_node,  # 如需使用，取消注释
    ])