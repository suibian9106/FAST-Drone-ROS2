from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 启动测试节点
        Node(
            package='test_px4ctrl',
            executable='px4ctrl_tester',
            name='px4ctrl_tester',
            output='screen'
        )
    ])