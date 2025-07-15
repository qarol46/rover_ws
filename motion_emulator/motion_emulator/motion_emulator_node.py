#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import argparse
from tf_transformations import euler_from_quaternion
import time
import threading

class MotionEmulator(Node):
    def __init__(self, motion_type, distance=None, angle=None, rectangle=None):
        super().__init__('motion_emulator')
        
        self.declare_parameter('linear_speed', 1.0)  # м/с
        self.declare_parameter('angular_speed', 0.75)  # рад/с
        self.declare_parameter('position_tolerance', 0.005)  # м
        self.declare_parameter('angle_tolerance', 0.005)  # рад
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.target_reached = False
        self.motion_type = motion_type
        self.custom_distance = distance
        self.custom_angle = angle
        self.rectangle_sides = rectangle
        self.rectangle_completed = False
        self.current_side = 0
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        if self.rectangle_sides:
            self.get_logger().info(f"Starting rectangle motion with sides: {self.rectangle_sides[0]}m x {self.rectangle_sides[1]}m")
        else:
            self.get_logger().info(f"Starting {motion_type} motion emulator")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        if self.initial_x is None:
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_yaw = self.current_yaw
            self.get_logger().info(
                f"Initial position captured: x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={math.degrees(self.initial_yaw):.1f}°"
            )

    def control_loop(self):
        if self.initial_x is None:
            return

        msg = Twist()
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        pos_tol = self.get_parameter('position_tolerance').value
        ang_tol = self.get_parameter('angle_tolerance').value

        if self.rectangle_sides and not self.rectangle_completed:
            self.handle_rectangle_motion(msg, linear_speed, angular_speed, pos_tol, ang_tol)
        elif self.motion_type == 'forward' and not self.target_reached:
            self.handle_forward_motion(msg, linear_speed, pos_tol)
        elif self.motion_type == 'turn' and not self.target_reached:
            self.handle_turn_motion(msg, angular_speed, ang_tol)

        self.cmd_pub.publish(msg)

        # Автоматическое завершение после достижения цели
        if self.target_reached or self.rectangle_completed:
            self.clean_shutdown()

    def handle_forward_motion(self, msg, linear_speed, pos_tol):
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        distance = math.sqrt(dx**2 + dy**2)
        target_distance = self.custom_distance if self.custom_distance is not None else 1.0
        
        if distance < target_distance - pos_tol:
            msg.linear.x = linear_speed
        else:
            msg.linear.x = 0.0
            self.target_reached = True
            self.get_logger().info(
                f"Target reached! Distance: {distance:.3f}m (target: {target_distance:.2f}m ±{pos_tol}m)"
            )

    def handle_turn_motion(self, msg, angular_speed, ang_tol):
        angle_diff = math.atan2(
            math.sin(self.current_yaw - self.initial_yaw),
            math.cos(self.current_yaw - self.initial_yaw)
        )
        target_angle = math.radians(self.custom_angle) if self.custom_angle is not None else math.pi/2
        
        if abs(angle_diff) < abs(target_angle) - ang_tol:
            msg.angular.z = angular_speed if target_angle > 0 else -angular_speed
        else:
            msg.angular.z = 0.0
            self.target_reached = True
            self.get_logger().info(
                f"Target reached! Angle: {math.degrees(abs(angle_diff)):.1f}° (target: {math.degrees(abs(target_angle)):.1f}° ±{math.degrees(ang_tol):.1f}°)"
            )

    def handle_rectangle_motion(self, msg, linear_speed, angular_speed, pos_tol, ang_tol):
        if self.current_side < 8:  # 4 стороны и 4 поворота
            if self.current_side % 2 == 0:  # Движение вперед
                start_x = self.initial_x if self.current_side == 0 else self.turn_start_x
                start_y = self.initial_y if self.current_side == 0 else self.turn_start_y
                
                dx = self.current_x - start_x
                dy = self.current_y - start_y
                distance = math.sqrt(dx**2 + dy**2)
                target_distance = self.rectangle_sides[0] if self.current_side in [0, 4] else self.rectangle_sides[1]
                
                if distance < target_distance - pos_tol:
                    msg.linear.x = linear_speed
                else:
                    msg.linear.x = 0.0
                    self.turn_start_x = self.current_x
                    self.turn_start_y = self.current_y
                    self.turn_start_yaw = self.current_yaw
                    self.current_side += 1
            else:  # Поворот
                angle_diff = math.atan2(
                    math.sin(self.current_yaw - self.turn_start_yaw),
                    math.cos(self.current_yaw - self.turn_start_yaw)
                )
                
                if abs(angle_diff) < math.pi/2 - ang_tol:
                    msg.angular.z = angular_speed
                else:
                    msg.angular.z = 0.0
                    self.current_side += 1
        else:
            self.rectangle_completed = True

    def clean_shutdown(self):
        # Остановка робота
        stop_msg = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Motion completed, shutting down...")
        
        # Завершение работы ноды в отдельном потоке
        def shutdown_thread():
            time.sleep(0.5)  # Даем время для завершения публикаций
            self.destroy_node()
            rclpy.shutdown()
        
        threading.Thread(target=shutdown_thread).start()

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Motion emulator for differential drive robot')
    parser.add_argument('motion', choices=['forward', 'turn', 'rectangle'], 
                       help='Type of motion: forward, turn, or rectangle')
    parser.add_argument('--distance', type=float, 
                       help='Distance to travel in meters (for forward motion)')
    parser.add_argument('--angle', type=float, 
                       help='Angle to turn in degrees (for turn motion)')
    parser.add_argument('--rectangle', nargs=2, type=float, metavar=('A', 'B'),
                       help='Rectangle sides A and B in meters')
    
    args = parser.parse_args()
    
    node = MotionEmulator(args.motion, args.distance, args.angle, args.rectangle)
    rclpy.spin(node)

if __name__ == '__main__':
    main()