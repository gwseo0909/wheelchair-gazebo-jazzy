#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class DrivingSupervisor(Node):

    def __init__(self):
        super().__init__('driving_supervisor')

        self.user_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_user',
            self.user_cmd_callback,
            10,
        )

        self.nav_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_cmd_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/model/wheelchair/cmd_vel',
            10,
        )

        self.goal_marker_pub = self.create_publisher(
            Marker,
            '/avoid_goal_marker',
            10,
        )

        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        self.front_distance = float('inf')
        self.state = 'MANUAL'

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0
        self.saved_heading = 0.0

        self.left_clearance = 0.0
        self.right_clearance = 0.0
        self.avoid_direction = None

        self.avoid_goal_x = 0.0
        self.avoid_goal_y = 0.0

        self.get_logger().info(
            'Driving supervisor started: '
            '/cmd_vel_user -> /model/wheelchair/cmd_vel'
        )

    def user_cmd_callback(self, msg):
        output_cmd = Twist()

        output_cmd.linear.x = msg.linear.x
        output_cmd.linear.y = msg.linear.y
        output_cmd.linear.z = msg.linear.z

        output_cmd.angular.x = msg.angular.x
        output_cmd.angular.y = msg.angular.y
        output_cmd.angular.z = msg.angular.z

        if 0.8 < self.front_distance <= 1.5 and msg.linear.x > 0.0:
            slowdown_ratio = (
                self.front_distance - 0.8
            ) / (
                1.5 - 0.8
            )

            output_cmd.linear.x = msg.linear.x * slowdown_ratio
        
        if self.state == 'AVOIDING_READY' and msg.linear.x > 0.0:
            output_cmd.linear.x = 0.0
            output_cmd.angular.z = 0.0

        self.cmd_pub.publish(output_cmd)


    def nav_cmd_callback(self, msg):
        if self.state == 'AVOIDING':
            self.cmd_pub.publish(msg)

    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.current_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )


    def publish_avoid_goal_marker(self):
        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'avoid_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.avoid_goal_x
        marker.pose.position.y = self.avoid_goal_y
        marker.pose.position.z = 0.2

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_marker_pub.publish(marker)


    def send_avoid_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.avoid_goal_x
        goal_msg.pose.pose.position.y = self.avoid_goal_y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(
            self.saved_heading / 2.0
        )
        goal_msg.pose.pose.orientation.w = math.cos(
            self.saved_heading / 2.0
        )

        self.get_logger().info(
            'Waiting for NavigateToPose action server...'
        )

        self.nav_to_pose_client.wait_for_server()

        self.get_logger().info(
            'Sending avoid goal to Nav2'
        )

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Avoid goal rejected')
            self.state = 'MANUAL'
            return

        self.get_logger().info('Avoid goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.goal_result_callback
        )

    def goal_result_callback(self, future):
        result = future.result()

        self.get_logger().info(
            f'Avoid goal finished. status={result.status}'
        )

        self.state = 'MANUAL'

    
    def scan_callback(self, msg):
        front_angle = math.radians(30.0)

        valid_ranges = []
        left_ranges = []
        right_ranges = []

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment

            if math.isfinite(distance):
                if msg.range_min <= distance <= msg.range_max:
                    if math.radians(30.0) < angle <= math.radians(90.0):
                        left_ranges.append(distance)

                    elif math.radians(-90.0) <= angle < math.radians(-30.0):
                        right_ranges.append(distance)

            if -front_angle <= angle <= front_angle:
                if math.isfinite(distance):
                    if msg.range_min <= distance <= msg.range_max:
                        valid_ranges.append(distance)

        if left_ranges:
            self.left_clearance = min(left_ranges)
        else:
            self.left_clearance = float('inf')

        if right_ranges:
            self.right_clearance = min(right_ranges)
        else:
            self.right_clearance = float('inf')

        if valid_ranges:
            front_distance = min(valid_ranges)
            self.front_distance = front_distance

            new_state = self.state

            if self.state == 'AVOIDING':
                new_state = 'AVOIDING'
            elif front_distance > 1.5:
                new_state = 'MANUAL'
            elif front_distance > 0.8:
                new_state = 'SLOWDOWN'
            else:
                new_state = 'AVOIDING_READY'

            if new_state != self.state:
                self.get_logger().info(
                    f'State changed: {self.state} -> {new_state}'
                )
                self.state = new_state

                if self.state == 'AVOIDING_READY':
                    self.avoid_start_x = self.current_x
                    self.avoid_start_y = self.current_y
                    self.saved_heading = self.current_yaw

                    if self.left_clearance > self.right_clearance:
                        self.avoid_direction = 'LEFT'
                    else:
                        self.avoid_direction = 'RIGHT'

                    forward_offset = 2.5
                    local_goal_y = 0.0

                    self.avoid_goal_x = (
                        self.current_x
                        + forward_offset * math.cos(self.saved_heading)
                        - local_goal_y * math.sin(self.saved_heading)
                    )

                    self.avoid_goal_y = (
                        self.current_y
                        + forward_offset * math.sin(self.saved_heading)
                        + local_goal_y * math.cos(self.saved_heading)
                    )

                    self.get_logger().info(
                        'Avoidance start saved: '
                        f'x={self.avoid_start_x:.2f}, '
                        f'y={self.avoid_start_y:.2f}, '
                        f'yaw={math.degrees(self.saved_heading):.1f} deg'
                    )

                    self.get_logger().info(
                        'Avoid direction selected: '
                        f'{self.avoid_direction} '
                        f'(left={self.left_clearance:.2f} m, '
                        f'right={self.right_clearance:.2f} m)'
                    )

                    self.get_logger().info(
                        'Avoid goal calculated: '
                        f'x={self.avoid_goal_x:.2f}, '
                        f'y={self.avoid_goal_y:.2f}'
                    )

                    self.publish_avoid_goal_marker()
                    self.send_avoid_goal()

                    self.state = 'AVOIDING'

                    stop_cmd = Twist()
                    self.cmd_pub.publish(stop_cmd)

            self.get_logger().info(
                f'Front obstacle distance: {front_distance:.2f} m'
            )
        else:
            self.get_logger().warn(
                'No valid LiDAR data in front area'
            )


def main(args=None):
    rclpy.init(args=args)

    node = DrivingSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()