#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener, TransformException
import math

class PencilTipTracer(Node):
    def __init__(self):
        super().__init__('pencil_tip_tracer')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Marker, '/pencil_tip_trace', 10)

        self.clear_previous_markers()

        self.delete_previous_board()

        self.current_marker_id = 0
        self.drawing_height = 0.04 # previous: 0.3
        self.error_margin = 0.003

        self.current_marker = self._create_new_marker()

        self.timer = self.create_timer(0.05, self.timer_callback)

    def clear_previous_markers(self):
        # Clear old markers
        delete_path_marker = Marker()
        delete_path_marker.header.frame_id = 'fr3_link0'
        delete_path_marker.ns = 'pencil_tip_path'
        delete_path_marker.id = 0
        delete_path_marker.action = Marker.DELETEALL
        self.publisher.publish(delete_path_marker)

    def _create_new_marker(self):
        marker = Marker()
        marker.header.frame_id = 'fr3_link0'
        marker.ns = 'pencil_tip_path'
        marker.id = self.current_marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.0007 # prev: 0.001
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0, nanosec=0)
        self.current_marker_id += 1
        return marker

    def is_jump(self, pt):

        return abs(pt.z - (self.drawing_height + self.raising_amount)) <= self.error_margin

    def timer_callback(self):
        self.publish_board()

        try:
            tf = self.buffer.lookup_transform('fr3_link0', 'pencil_tip', rclpy.time.Time())

            pt = Point()
            pt.x = tf.transform.translation.x
            pt.y = tf.transform.translation.y
            pt.z = tf.transform.translation.z
            
            if abs(pt.z - self.drawing_height) <= self.error_margin:
                    self.current_marker.points.append(pt)
            else:
                # Publish the old marker
                self.current_marker.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(self.current_marker)

                # Start a new marker
                self.current_marker = self._create_new_marker()

            # Always publish the active marker too
            self.current_marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.current_marker)

            self.get_logger().info(f'Tip at ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})')

        except TransformException as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')

    def delete_previous_board(self):
        # Remove Old Board
        delete_plane_marker = Marker()
        delete_plane_marker.header.frame_id = 'base'
        delete_plane_marker.ns = 'white_plane'
        delete_plane_marker.id = 0
        delete_plane_marker.action = Marker.DELETE
        self.publisher.publish(delete_plane_marker)

    def publish_board(self):
        marker = Marker()
        marker.header.frame_id = 'fr3_link0'
        marker.ns = 'white_plane'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.scale.x = 1.3  # width
        marker.scale.y = 1.3  # height
        marker.scale.z = 0.001  # very thin plane

        # correcting_factor = 0.006
        marker.pose.position.x = 0.85
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 0.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # fully opaque

        self.publisher.publish(marker)

def main():
    rclpy.init()
    node = PencilTipTracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
