from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 定义启动参数（可选，如果需要从命令行覆盖参数）
    time_interval_arg = DeclareLaunchArgument(
        'time_interval',
        default_value='1.0',
        description='Time interval for thrust calibration'
    )
    
    mass_kg_arg = DeclareLaunchArgument(
        'mass_kg',
        default_value='10.7',
        description='Mass in kg for thrust calibration'
    )
    
    # 定义thrust_calibrate节点
    thrust_calibrate_node = Node(
        package='px4ctrl',
        executable='thrust_calibrate.py',
        name='thrust_calibrate',
        output='screen',
        parameters=[{
            'time_interval': LaunchConfiguration('time_interval'),
            'mass_kg': LaunchConfiguration('mass_kg'),
        }]
    )

    return LaunchDescription([
        time_interval_arg,
        mass_kg_arg,
        thrust_calibrate_node,
    ])