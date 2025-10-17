#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, Joy
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

class OdometryConverter:
    def __init__(self, node, frame_id_in_, frame_id_out_, broadcast_tf_, 
                 body_frame_id_, intermediate_frame_id_, world_frame_id_):
        self.node = node
        self.frame_id_in = frame_id_in_
        self.frame_id_out = frame_id_out_
        self.broadcast_tf = broadcast_tf_
        self.body_frame_id = body_frame_id_
        self.intermediate_frame_id = intermediate_frame_id_
        self.world_frame_id = world_frame_id_
        
        self.tf_pub_flag = True
        if self.broadcast_tf:
            self.node.get_logger().info(
                f'ROSTopic: [{self.frame_id_in}]->[{self.frame_id_out}] '
                f'TF: [{self.body_frame_id}]-[{self.intermediate_frame_id}]-[{self.world_frame_id}]')
        else:
            self.node.get_logger().info(
                f'ROSTopic: [{self.frame_id_in}]->[{self.frame_id_out}] No TF')

        self.path = []
        self.tf_broadcaster = TransformBroadcaster(self.node)

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = euler_from_quaternion(q, 'rzyx')
        wqb = quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = quaternion_from_euler(e[0],  0.0,  0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        assert(in_odom_msg.header.frame_id == self.frame_id_in)
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            
            if not self.frame_id_in == self.frame_id_out:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = self.frame_id_in
                t.child_frame_id = self.frame_id_out
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

            if not self.world_frame_id == self.frame_id_out:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = self.world_frame_id
                t.child_frame_id = self.frame_id_out
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

            # Body transform
            t_body = TransformStamped()
            t_body.header.stamp = odom_msg.header.stamp
            t_body.header.frame_id = self.world_frame_id
            t_body.child_frame_id = self.body_frame_id
            t_body.transform.translation.x = p[0]
            t_body.transform.translation.y = p[1]
            t_body.transform.translation.z = p[2]
            t_body.transform.rotation.x = wqb[0]
            t_body.transform.rotation.y = wqb[1]
            t_body.transform.rotation.z = wqb[2]
            t_body.transform.rotation.w = wqb[3]
            self.tf_broadcaster.sendTransform(t_body)

            # Intermediate transform
            t_inter = TransformStamped()
            t_inter.header.stamp = odom_msg.header.stamp
            t_inter.header.frame_id = self.world_frame_id
            t_inter.child_frame_id = self.intermediate_frame_id
            t_inter.transform.translation.x = p[0]
            t_inter.transform.translation.y = p[1]
            t_inter.transform.translation.z = p[2]
            t_inter.transform.rotation.x = wqc[0]
            t_inter.transform.rotation.y = wqc[1]
            t_inter.transform.rotation.z = wqc[2]
            t_inter.transform.rotation.w = wqc[3]
            self.tf_broadcaster.sendTransform(t_inter)

        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True

class TfAssist(Node):
    def __init__(self):
        super().__init__('tf_assist')
        
        self.converters = []
        index = 0
        
        while True:
            prefix = f"converter{index}."
            try:
                frame_id_in = self.declare_parameter(f'{prefix}frame_id_in', '').value
                if not frame_id_in:  # Empty string means no more converters
                    break
                    
                frame_id_out = self.declare_parameter(f'{prefix}frame_id_out', '').value
                broadcast_tf = self.declare_parameter(f'{prefix}broadcast_tf', False).value
                body_frame_id = self.declare_parameter(f'{prefix}body_frame_id', 'body').value
                intermediate_frame_id = self.declare_parameter(
                    f'{prefix}intermediate_frame_id', 'intermediate').value
                world_frame_id = self.declare_parameter(
                    f'{prefix}world_frame_id', 'world').value

                converter = OdometryConverter(
                    self, frame_id_in, frame_id_out, broadcast_tf, 
                    body_frame_id, intermediate_frame_id, world_frame_id)
                    
                converter.in_odom_sub = self.create_subscription(
                    Odometry, f'{prefix}in_odom', converter.in_odom_callback, 10)
                    
                converter.out_odom_pub = self.create_publisher(
                    Odometry, f'{prefix}out_odom', 10)
                    
                converter.out_path_pub = self.create_publisher(
                    Path, f'{prefix}out_path', 10)

                converter.tf_pub_timer = self.create_timer(0.1, converter.tf_pub_callback)
                converter.path_pub_timer = self.create_timer(0.5, converter.path_pub_callback)

                self.converters.append(converter)
                index += 1
                
            except Exception as e:
                if index == 0:
                    raise e
                else:
                    self.get_logger().info(
                        f'prefix:"{prefix}" not found. Generate {index} converter(s).')
                    break

def main(args=None):
    rclpy.init(args=args)
    node = TfAssist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()