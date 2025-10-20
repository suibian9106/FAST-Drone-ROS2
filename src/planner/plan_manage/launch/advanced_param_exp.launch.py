import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare all launch arguments
    launch_arguments = []
    
    # Map size parameters
    launch_arguments.append(DeclareLaunchArgument('map_size_x_', description='Map size in x direction'))
    launch_arguments.append(DeclareLaunchArgument('map_size_y_', description='Map size in y direction'))
    launch_arguments.append(DeclareLaunchArgument('map_size_z_', description='Map size in z direction'))
    
    # Topic parameters
    launch_arguments.append(DeclareLaunchArgument('odometry_topic', description='Odometry topic'))
    launch_arguments.append(DeclareLaunchArgument('camera_pose_topic', description='Camera pose topic'))
    launch_arguments.append(DeclareLaunchArgument('depth_topic', description='Depth image topic'))
    launch_arguments.append(DeclareLaunchArgument('cloud_topic', description='Point cloud topic'))
    
    # Camera intrinsic parameters
    launch_arguments.append(DeclareLaunchArgument('cx', description='Camera cx parameter'))
    launch_arguments.append(DeclareLaunchArgument('cy', description='Camera cy parameter'))
    launch_arguments.append(DeclareLaunchArgument('fx', description='Camera fx parameter'))
    launch_arguments.append(DeclareLaunchArgument('fy', description='Camera fy parameter'))
    
    # Planning parameters
    launch_arguments.append(DeclareLaunchArgument('max_vel', description='Maximum velocity'))
    launch_arguments.append(DeclareLaunchArgument('max_acc', description='Maximum acceleration'))
    launch_arguments.append(DeclareLaunchArgument('planning_horizon', description='Planning horizon'))
    
    # Waypoint parameters
    launch_arguments.append(DeclareLaunchArgument('point_num', description='Number of waypoints'))
    launch_arguments.append(DeclareLaunchArgument('point0_x', description='Waypoint 0 x'))
    launch_arguments.append(DeclareLaunchArgument('point0_y', description='Waypoint 0 y'))
    launch_arguments.append(DeclareLaunchArgument('point0_z', description='Waypoint 0 z'))
    launch_arguments.append(DeclareLaunchArgument('point1_x', description='Waypoint 1 x'))
    launch_arguments.append(DeclareLaunchArgument('point1_y', description='Waypoint 1 y'))
    launch_arguments.append(DeclareLaunchArgument('point1_z', description='Waypoint 1 z'))
    launch_arguments.append(DeclareLaunchArgument('point2_x', description='Waypoint 2 x'))
    launch_arguments.append(DeclareLaunchArgument('point2_y', description='Waypoint 2 y'))
    launch_arguments.append(DeclareLaunchArgument('point2_z', description='Waypoint 2 z'))
    launch_arguments.append(DeclareLaunchArgument('point3_x', description='Waypoint 3 x'))
    launch_arguments.append(DeclareLaunchArgument('point3_y', description='Waypoint 3 y'))
    launch_arguments.append(DeclareLaunchArgument('point3_z', description='Waypoint 3 z'))
    launch_arguments.append(DeclareLaunchArgument('point4_x', description='Waypoint 4 x'))
    launch_arguments.append(DeclareLaunchArgument('point4_y', description='Waypoint 4 y'))
    launch_arguments.append(DeclareLaunchArgument('point4_z', description='Waypoint 4 z'))
    
    # Flight type and trajectory parameters
    launch_arguments.append(DeclareLaunchArgument('flight_type', description='Flight type'))
    launch_arguments.append(DeclareLaunchArgument('use_distinctive_trajs', description='Use distinctive trajectories'))
    
    # Object and drone parameters
    launch_arguments.append(DeclareLaunchArgument('obj_num_set', description='Number of objects'))
    launch_arguments.append(DeclareLaunchArgument('drone_id', description='Drone ID'))
    
    # Ego planner node
    drone_id_str = LaunchConfiguration('drone_id')
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id_str, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('~/odom_world', LaunchConfiguration('odometry_topic')),
            ('~/planning/bspline', ['/drone_', drone_id_str, '_planning/bspline']),
            ('~/planning/data_display', ['/drone_', drone_id_str, '_planning/data_display']),
            ('~/planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('~/planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            ('~/grid_map/odom', LaunchConfiguration('odometry_topic')),
            ('~/grid_map/cloud', LaunchConfiguration('cloud_topic')),
            ('~/grid_map/pose', LaunchConfiguration('camera_pose_topic')),
            ('~/grid_map/depth', LaunchConfiguration('depth_topic'))
        ],
        parameters=[{
            # Planning FSM parameters
            'fsm.flight_type': LaunchConfiguration('flight_type'),
            'fsm.thresh_replan_time': 1.0,
            'fsm.thresh_no_replan_meter': 1.0,
            'fsm.planning_horizon': LaunchConfiguration('planning_horizon'),
            'fsm.planning_horizen_time': 3.0,
            'fsm.emergency_time': 1.0,
            'fsm.realworld_experiment': True,
            'fsm.fail_safe': True,
            
            # Waypoint parameters
            'fsm.waypoint_num': LaunchConfiguration('point_num'),
            'fsm.waypoint0_x': LaunchConfiguration('point0_x'),
            'fsm.waypoint0_y': LaunchConfiguration('point0_y'),
            'fsm.waypoint0_z': LaunchConfiguration('point0_z'),
            'fsm.waypoint1_x': LaunchConfiguration('point1_x'),
            'fsm.waypoint1_y': LaunchConfiguration('point1_y'),
            'fsm.waypoint1_z': LaunchConfiguration('point1_z'),
            'fsm.waypoint2_x': LaunchConfiguration('point2_x'),
            'fsm.waypoint2_y': LaunchConfiguration('point2_y'),
            'fsm.waypoint2_z': LaunchConfiguration('point2_z'),
            'fsm.waypoint3_x': LaunchConfiguration('point3_x'),
            'fsm.waypoint3_y': LaunchConfiguration('point3_y'),
            'fsm.waypoint3_z': LaunchConfiguration('point3_z'),
            'fsm.waypoint4_x': LaunchConfiguration('point4_x'),
            'fsm.waypoint4_y': LaunchConfiguration('point4_y'),
            'fsm.waypoint4_z': LaunchConfiguration('point4_z'),
            
            # Grid map parameters
            'grid_map.resolution': 0.15,
            'grid_map.map_size_x': LaunchConfiguration('map_size_x_'),
            'grid_map.map_size_y': LaunchConfiguration('map_size_y_'),
            'grid_map.map_size_z': LaunchConfiguration('map_size_z_'),
            'grid_map.local_update_range_x': 5.5,
            'grid_map.local_update_range_y': 5.5,
            'grid_map.local_update_range_z': 4.5,
            'grid_map.obstacles_inflation': 0.299,
            'grid_map.local_map_margin': 10,
            'grid_map.ground_height': -0.01,
            
            # Camera parameters
            'grid_map.cx': LaunchConfiguration('cx'),
            'grid_map.cy': LaunchConfiguration('cy'),
            'grid_map.fx': LaunchConfiguration('fx'),
            'grid_map.fy': LaunchConfiguration('fy'),
            
            # Depth filter parameters
            'grid_map.use_depth_filter': True,
            'grid_map.depth_filter_tolerance': 0.15,
            'grid_map.depth_filter_maxdist': 5.0,
            'grid_map.depth_filter_mindist': 0.2,
            'grid_map.depth_filter_margin': 2,
            'grid_map.k_depth_scaling_factor': 1000.0,
            'grid_map.skip_pixel': 2,
            
            # Local fusion parameters
            'grid_map.p_hit': 0.65,
            'grid_map.p_miss': 0.35,
            'grid_map.p_min': 0.12,
            'grid_map.p_max': 0.90,
            'grid_map.p_occ': 0.80,
            'grid_map.min_ray_length': 0.3,
            'grid_map.max_ray_length': 5.0,
            
            # Visualization and frame parameters
            'grid_map.visualization_truncate_height': 1.8,
            'grid_map.show_occ_time': False,
            'grid_map.pose_type': 2,
            'grid_map.frame_id': 'world',
            
            # Planner manager parameters
            'manager.max_vel': LaunchConfiguration('max_vel'),
            'manager.max_acc': LaunchConfiguration('max_acc'),
            'manager.max_jerk': 4.0,
            'manager.control_points_distance': 0.4,
            'manager.feasibility_tolerance': 0.05,
            'manager.planning_horizon': LaunchConfiguration('planning_horizon'),
            'manager.use_distinctive_trajs': LaunchConfiguration('use_distinctive_trajs'),
            'manager.drone_id': LaunchConfiguration('drone_id'),
            
            # Trajectory optimization parameters
            'optimization.lambda_smooth': 1.0,
            'optimization.lambda_collision': 0.5,
            'optimization.lambda_feasibility': 0.1,
            'optimization.lambda_fitness': 1.0,
            'optimization.dist0': 0.5,
            'optimization.swarm_clearance': 0.5,
            'optimization.max_vel': LaunchConfiguration('max_vel'),
            'optimization.max_acc': LaunchConfiguration('max_acc'),
            
            # B-spline parameters
            'bspline.limit_vel': LaunchConfiguration('max_vel'),
            'bspline.limit_acc': LaunchConfiguration('max_acc'),
            'bspline.limit_ratio': 1.1,
            
            # Object prediction parameters
            'prediction.obj_num': LaunchConfiguration('obj_num_set'),
            'prediction.lambda': 1.0,
            'prediction.predict_rate': 1.0
        }]
    )
    
    launch_arguments.append(ego_planner_node)
    return LaunchDescription(launch_arguments)