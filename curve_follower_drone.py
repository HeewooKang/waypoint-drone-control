import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped


class CurveFollowerDrone(Node):
    def __init__(self):
        super().__init__('curve_follower_drone')

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

        self.kp = 1.0
        self.max_lin = 1.2
        self.acc = 0.25

        self.flight_z = 2.0
        self.takeoff_tolerance = 0.15
        self.mode = 'takeoff'

        self.t = 0.0
        self.dt = 0.02
        self.t_max = 20.0

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Curve follower node started')

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

    # -----------------------------
    # 여기만 바꾸면 다른 곡선도 가능
    # -----------------------------
    def curve_function(self, t):
        x = 2.0 * math.sin(t)
        y = 1.0 * math.sin(2.0 * t)
        z = self.flight_z
        return x, y, z

    def control_loop(self):
        if not self.pose_received:
            self.get_logger().info(
                'Waiting for /mavros/local_position/pose ...',
                throttle_duration_sec=2.0
            )
            return

        if self.mode == 'takeoff':
            error_z = self.flight_z - self.current_z

            if abs(error_z) < self.takeoff_tolerance:
                self.mode = 'follow_curve'
                self.t = 0.0
                self.get_logger().info('Takeoff complete. Start curve following.')
                return

            tx = 0.0
            ty = 0.0
            tz = 0.8 * error_z
            tz = self.clamp(tz, -0.6, 0.6)

            self.vx = self.smooth_step(self.vx, tx)
            self.vy = self.smooth_step(self.vy, ty)
            self.vz = self.smooth_step(self.vz, tz)
            self.wz = 0.0

            self.publish_cmd()
            return

        if self.mode == 'follow_curve':
            if self.t > self.t_max:
                self.stop_drone()
                self.get_logger().info('Curve following complete!', throttle_duration_sec=2.0)
                return

            target_x, target_y, target_z = self.curve_function(self.t)

            error_x = target_x - self.current_x
            error_y = target_y - self.current_y
            error_z = target_z - self.current_z

            tx = self.kp * error_x
            ty = self.kp * error_y
            tz = self.kp * error_z

            tx = self.clamp(tx, -self.max_lin, self.max_lin)
            ty = self.clamp(ty, -self.max_lin, self.max_lin)
            tz = self.clamp(tz, -0.8, 0.8)

            self.vx = self.smooth_step(self.vx, tx)
            self.vy = self.smooth_step(self.vy, ty)
            self.vz = self.smooth_step(self.vz, tz)
            self.wz = 0.0

            self.publish_cmd()

            self.get_logger().info(
                f't={self.t:.2f} '
                f'current=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
                f'target=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) '
                f'vel=({self.vx:.2f}, {self.vy:.2f}, {self.vz:.2f})',
                throttle_duration_sec=0.5
            )

            self.t += self.dt

    def stop_and_cleanup(self):
        self.stop_drone()


def main():
    rclpy.init()
    node = CurveFollowerDrone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_cleanup()
        node.destroy_node()


if __name__ == '__main__':
    main()
