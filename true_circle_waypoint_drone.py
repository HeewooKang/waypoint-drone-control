import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped


class TrueCircleDrone(Node):
    def __init__(self):
        super().__init__('true_circle_drone')

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

        self.center_x = 0.0
        self.center_y = 0.0
        self.radius = 2.0
        self.target_z = 2.0

        self.tangent_speed = 1.5     # 원을 도는 기본 속도
        self.radial_kp = 1.2         # 반지름 오차 보정
        self.altitude_kp = 0.8       # 고도 유지
        self.max_lin = 1.2           # 최대 선속도
        self.acc = 0.3               # 부드럽게 적용

        self.mode = 'takeoff'        
        self.takeoff_tolerance = 0.15

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('True circle flight node started')

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

        if self.mode == 'takeoff':
            error_z = self.target_z - self.current_z

            if abs(error_z) < self.takeoff_tolerance:
                self.mode = 'circle'
                self.get_logger().info('Takeoff complete. Start circle flight.')
                return

            tx = 0.0
            ty = 0.0
            tz = self.altitude_kp * error_z

            tz = self.clamp(tz, -0.6, 0.6)

            self.vx = self.smooth_step(self.vx, tx)
            self.vy = self.smooth_step(self.vy, ty)
            self.vz = self.smooth_step(self.vz, tz)
            self.wz = 0.0

            self.publish_cmd()
            return

        dx = self.current_x - self.center_x
        dy = self.current_y - self.center_y

        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.001:
            dist = 0.001

        ux = dx / dist
        uy = dy / dist

        tx_tan = -uy
        ty_tan = ux

        radius_error = self.radius - dist

        target_vx = self.tangent_speed * tx_tan - self.radial_kp * (-radius_error * ux)
        target_vy = self.tangent_speed * ty_tan - self.radial_kp * (-radius_error * uy)

        error_z = self.target_z - self.current_z
        target_vz = self.altitude_kp * error_z

        target_vx = self.clamp(target_vx, -self.max_lin, self.max_lin)
        target_vy = self.clamp(target_vy, -self.max_lin, self.max_lin)
        target_vz = self.clamp(target_vz, -0.6, 0.6)

        self.vx = self.smooth_step(self.vx, target_vx)
        self.vy = self.smooth_step(self.vy, target_vy)
        self.vz = self.smooth_step(self.vz, target_vz)
        self.wz = 0.0

        self.publish_cmd()

        self.get_logger().info(
            f'pos=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
            f'dist={dist:.2f} radius_err={radius_error:.2f} '
            f'vel=({self.vx:.2f}, {self.vy:.2f}, {self.vz:.2f})',
            throttle_duration_sec=0.5
        )

    def stop_and_cleanup(self):
        self.stop_drone()

def main():
    rclpy.init()
    node = TrueCircleDrone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_cleanup()
        node.destroy_node()

if __name__ == '__main__':
    main()
