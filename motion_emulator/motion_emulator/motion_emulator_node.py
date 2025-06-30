#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import argparse
from tf_transformations import euler_from_quaternion

class MotionEmulator(Node):
    def __init__(self, motion_type):
        super().__init__('motion_emulator')
        
        # Параметры движения с значениями по умолчанию
        self.declare_parameter('linear_speed', 0.2)  # м/с
        self.declare_parameter('angular_speed', 0.5)  # рад/с
        self.declare_parameter('position_tolerance', 0.05)  # м
        self.declare_parameter('angle_tolerance', 0.05)  # рад
        self.declare_parameter('turn_correction_factor', 1.8)  # Поправочный коэффициент для поворотов
        
        # Текущее состояние
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.target_reached = False
        self.motion_type = motion_type
        
        # Публикатор команд
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Подписка на одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.odom_callback,
            10)
        
        # Таймер управления
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Starting {motion_type} motion emulator")

    def odom_callback(self, msg):
        # Обновляем текущую позицию
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Получаем ориентацию из кватерниона
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        # Инициализируем начальные значения при первом вызове
        if self.initial_x is None:
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_yaw = self.current_yaw
            self.get_logger().info(
                f"Initial position captured: x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={math.degrees(self.initial_yaw):.1f}°"
            )

    def control_loop(self):
        if self.initial_x is None or self.target_reached:
            return

        msg = Twist()
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        pos_tol = self.get_parameter('position_tolerance').value
        ang_tol = self.get_parameter('angle_tolerance').value
        turn_correction = self.get_parameter('turn_correction_factor').value

        if self.motion_type == 'forward':
            # Расчет пройденного расстояния по прямой (евклидово расстояние)
            dx = self.current_x - self.initial_x
            dy = self.current_y - self.initial_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 1.0 - pos_tol:
                msg.linear.x = linear_speed
            else:
                msg.linear.x = 0.0
                self.target_reached = True
                self.get_logger().info(
                    f"Target reached! Distance: {distance:.3f}m (target: 1.0m ±{pos_tol}m)"
                )
                self.clean_shutdown()

        elif self.motion_type == 'turn':
            # Расчет угла поворота с поправочным коэффициентом
            angle_diff = math.atan2(
                math.sin(self.current_yaw - self.initial_yaw),
                math.cos(self.current_yaw - self.initial_yaw)
            )
            
            if abs(angle_diff) < (math.pi/2 * turn_correction) - ang_tol:
                # Применяем направление поворота с учетом знака угла
                msg.angular.z = angular_speed
            else:
                msg.angular.z = 0.0
                self.target_reached = True
                self.get_logger().info(
                    f"Target reached! Angle: {math.degrees(abs(angle_diff)):.1f}° (target: {90*turn_correction:.1f}° ±{math.degrees(ang_tol):.1f}°)"
                )
                self.clean_shutdown()

        self.cmd_pub.publish(msg)

    def clean_shutdown(self):
        # Гарантированная остановка
        stop_msg = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Motion completed, shutting down...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Motion emulator for differential drive robot')
    parser.add_argument('motion', choices=['forward', 'turn'], 
                       help='Type of motion: forward (1m) or turn (90 degrees)')
    
    # Парсинг аргументов
    args, _ = parser.parse_known_args()
    
    try:
        node = MotionEmulator(args.motion)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()