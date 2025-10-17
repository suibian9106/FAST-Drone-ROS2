from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 获取px4ctrl功能包的共享目录路径，用于定位参数文件
    px4ctrl_pkg_dir = get_package_share_directory('px4ctrl')
    
    # 构建参数文件的完整路径
    param_file_path = os.path.join(px4ctrl_pkg_dir, 'config', 'ctrl_param_fpv.yaml')
    
    # 确保参数文件存在
    if not os.path.exists(param_file_path):
        raise FileNotFoundError(f"Parameter file not found: {param_file_path}")
    
    # 定义px4ctrl节点
    px4ctrl_node = Node(
        package='px4ctrl',
        executable='px4ctrl_node',
        name='px4ctrl',
        output='screen',
        remappings=[
            # 在ROS2中，通常使用以'/'开头的全局话题名，或者与节点名配合使用私有命名空间。
            # 原 '~odom' 通常映射为 '/vins_fusion/imu_propagate'
            ('odom', '/vins_fusion/imu_propagate'),
            # 原 '~cmd' 映射为 '/position_cmd'
            ('cmd', '/position_cmd'),
        ],
        # 加载参数文件
        parameters=[param_file_path]
    )

    return LaunchDescription([
        px4ctrl_node,
    ])