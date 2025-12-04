#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import Jetson.GPIO as GPIO

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')


        self.cpr_left = 972.0        
        self.cpr_right = 964.0       

        self.wheel_diameter = 0.067  
        self.wheel_base = 0.19       

        # circunferencia 
        self.wheel_circ = math.pi * self.wheel_diameter

        # PINES DE LOS ENCODERS
        self.left_pin_a = 11
        self.left_pin_b = 13
        self.right_pin_a = 15
        self.right_pin_b = 16

        GPIO.setmode(GPIO.BOARD) 

        GPIO.setup(self.left_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.left_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Variables
        self.prev_left = 0
        self.prev_right = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_time = self.get_clock().now()

        # PUB
        self.pub_odom = self.create_publisher(Odometry, 'demo/odom', 10)

        # 50 Hz, 20 ms
        self.timer = self.create_timer(0.02, self.update_odom)

        # Cont encoder
        self.left_ticks = 0
        self.right_ticks = 0

        # interrupción 
        GPIO.add_event_detect(self.left_pin_a, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.right_pin_a, GPIO.RISING, callback=self.right_encoder_callback)

    # Odometria
    def update_odom(self):
        # Leer ticks 
        ticks_left = self.left_ticks
        ticks_right = self.right_ticks
        print("ticks_left:", ticks_left, "ticks_right:", ticks_right)

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0:
            return

        dL_ticks = ticks_left - self.prev_left
        dR_ticks = ticks_right - self.prev_right

        self.prev_left = ticks_left
        self.prev_right = ticks_right

        # DISTANCIA DE RUEDA 
        dist_left = (dL_ticks / self.cpr_left) * self.wheel_circ
        dist_right = (dR_ticks / self.cpr_right) * self.wheel_circ

        # DIFERENCIAL 
        dist_center = (dist_left + dist_right) / 2.0
        dtheta = 0.0
        if self.wheel_base != 0:
            dtheta = (dist_right - dist_left) / self.wheel_base

        # INTEGRACIÓN 
        self.x += dist_center * math.cos(self.theta + dtheta / 2.0)
        self.y += dist_center * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # PUB ODOM 
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # orientación
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)

        # velocidades
        odom.twist.twist.linear.x = dist_center / dt
        odom.twist.twist.angular.z = dtheta / dt if dt > 0 else 0.0

        self.pub_odom.publish(odom)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q


    def left_encoder_callback(self, channel):
        self.left_ticks += 1

    def right_encoder_callback(self, channel):
        self.right_ticks += 1

    def on_shutdown(self):
        """Limpieza de GPIO al cerrar el nodo."""
        GPIO.cleanup()  # Limpiar pines al terminar
        self.get_logger().info('Limpiando GPIO')


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido')
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
