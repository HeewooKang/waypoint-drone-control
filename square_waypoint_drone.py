import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped


class SquareWaypointDrone(Node):
    def __init__(self):
        super().__init__('square_waypoint_drone')

        self.pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            pose_qos
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.pose_received = False

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0

        self.kp = 0.6
        self.max_lin = 0.5
        self.acc = 1.0
        self.tolerance = 0.2

        self.waypoints = [
            (0.0, 0.0, 3.0),
            (3.0, 0.0, 3.0),
            (3.0, 3.0, 3.0),
            (0.0, 3.0, 3.0),
            (0.0, 0.0, 3.0),
            (0.0, 0.0, 0.0)
        ]
        self.current_wp_index = 0

        self.get_logger().info('Square flight node started')
        self.get_logger().info(f'Current target: {self.waypoints[self.current_wp_index]}')

        self.timer = self.create_timer(0.02, self.control_loop)

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.pose_received = True

    def smooth_step(self, current, target):
        return current + (target - current) * self.acc

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz
        msg.twist.angular.z = self.wz
        self.pub.publish(msg)

    def stop_drone(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0
        self.publish_cmd()

    def control_loop(self):
        if not self.pose_received:
            self.get_logger().info(
                'Waiting for /mavros/local_position/pose ...',
                throttle_duration_sec=2.0
            )
            return

        if self.current_wp_index >= len(self.waypoints):
            self.stop_drone()
            self.get_logger().info('Square flight complete!', throttle_duration_sec=2.0)
            return

        target_x, target_y, target_z = self.waypoints[self.current_wp_index]

        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        error_z = target_z - self.current_z

        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        if distance < self.tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_wp_index} reached: '
                f'({target_x:.2f}, {target_y:.2f}, {target_z:.2f})'
            )
            self.current_wp_index += 1

            if self.current_wp_index < len(self.waypoints):
                self.get_logger().info(
                    f'Next target: {self.waypoints[self.current_wp_index]}'
                )
            else:
                self.stop_drone()
            return

        tx = self.kp * error_x
        ty = self.kp * error_y
        tz = self.kp * error_z

        tx = self.clamp(tx, -self.max_lin, self.max_lin)
        ty = self.clamp(ty, -self.max_lin, self.max_lin)
        tz = self.clamp(tz, -self.max_lin, self.max_lin)

        self.vx = self.smooth_step(self.vx, tx)
        self.vy = self.smooth_step(self.vy, ty)
        self.vz = self.smooth_step(self.vz, tz)
        self.wz = 0.0

        self.publish_cmd()

        self.get_logger().info(
            f'current=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
            f'target=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) '
            f'vel=({self.vx:.2f}, {self.vy:.2f}, {self.vz:.2f})',
            throttle_duration_sec=0.5
        )

    def stop_and_cleanup(self):
        self.stop_drone()


def main():
    rclpy.init()
    node = SquareWaypointDrone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_cleanup()
        node.destroy_node()


if __name__ == '__main__':
    main()
