import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取vins功能包的共享目录路径
    vins_pkg_path = get_package_share_directory('vins')
    config_path = os.path.join(vins_pkg_path, './config/fast_drone_250.yaml')
    
    # 确保使用绝对路径
    config_path = os.path.abspath(config_path)
    
    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_fusion',
        output='screen',
        arguments=[config_path]
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