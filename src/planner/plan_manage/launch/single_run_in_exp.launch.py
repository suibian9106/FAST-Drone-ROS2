import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch arguments
    obj_num = DeclareLaunchArgument('obj_num', default_value='10')
    drone_id = DeclareLaunchArgument('drone_id', default_value='0')
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='100')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='50')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='3.0')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='/vins_fusion/imu_propagate')
    
    # Include main algorithm parameters
    advanced_params_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ego_planner'),
                'launch',
                'advanced_param_exp.launch.py'
            ])
        ]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'map_size_x_': LaunchConfiguration('map_size_x'),
            'map_size_y_': LaunchConfiguration('map_size_y'),
            'map_size_z_': LaunchConfiguration('map_size_z'),
            'odometry_topic': LaunchConfiguration('odom_topic'),
            'obj_num_set': LaunchConfiguration('obj_num'),
            'camera_pose_topic': 'nouse1',
            'depth_topic': '/camera/depth/image_rect_raw',
            'cloud_topic': 'nouse2',
            'cx': '323.3316345214844',
            'cy': '234.95498657226562',
            'fx': '384.39654541015625',
            'fy': '384.39654541015625',
            'max_vel': '0.5',
            'max_acc': '6.0',
            'planning_horizon': '6',
            'use_distinctive_trajs': 'false',
            'flight_type': '1',
            'point_num': '1',
            'point0_x': '15',
            'point0_y': '0',
            'point0_z': '1',
            'point1_x': '0.0',
            'point1_y': '0.0',
            'point1_z': '1.0',
            'point2_x': '15.0',
            'point2_y': '0.0',
            'point2_z': '1.0',
            'point3_x': '0.0',
            'point3_y': '0.0',
            'point3_z': '1.0',
            'point4_x': '15.0',
            'point4_y': '0.0',
            'point4_z': '1.0'
        }.items()
    )
    
    # Trajectory server node
    drone_id_str = LaunchConfiguration('drone_id')
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id_str, '_traj_server'],
        output='screen',
        remappings=[
            # ('position_cmd', '/setpoints_cmd'),
            ('~/planning/bspline', ['drone_', drone_id_str, '_planning/bspline'])
        ],
        parameters=[{
            'traj_server/time_forward': 1.0
        }]
    )
    
    return LaunchDescription([
        obj_num,
        drone_id,
        map_size_x,
        map_size_y,
        map_size_z,
        odom_topic,
        advanced_params_launch,
        traj_server_node
    ])