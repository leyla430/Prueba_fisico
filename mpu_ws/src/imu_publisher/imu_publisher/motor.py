#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import Jetson.GPIO as GPIO

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info('Nodo MotorController iniciado.')

        self.declare_parameter('max_speed', 0.5)      
        self.declare_parameter('max_angular_speed', 1.5) 

        # Pines 
        self.declare_parameter('right_pwm_pin', 32)  
        self.declare_parameter('right_in1_pin', 36)  
        self.declare_parameter('right_in2_pin', 38) 
        self.declare_parameter('invert_right_motor', False) # if  polaridad  invertida 

        # Pines 
        self.declare_parameter('left_pwm_pin', 33)   
        self.declare_parameter('left_in1_pin', 37) 
        self.declare_parameter('left_in2_pin', 35)   
        self.declare_parameter('invert_left_motor', False)  # if  polaridad  invertida


        self.MAX_LINEAR_SPEED = self.get_parameter('max_speed').get_parameter_value().double_value
        self.MAX_ANGULAR_SPEED = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.INVERT_RIGHT = self.get_parameter('invert_right_motor').get_parameter_value().bool_value
        self.INVERT_LEFT = self.get_parameter('invert_left_motor').get_parameter_value().bool_value
        

        R_PWM = self.get_parameter('right_pwm_pin').get_parameter_value().integer_value
        R_IN1 = self.get_parameter('right_in1_pin').get_parameter_value().integer_value
        R_IN2 = self.get_parameter('right_in2_pin').get_parameter_value().integer_value
        
        L_PWM = self.get_parameter('left_pwm_pin').get_parameter_value().integer_value
        L_IN1 = self.get_parameter('left_in1_pin').get_parameter_value().integer_value
        L_IN2 = self.get_parameter('left_in2_pin').get_parameter_value().integer_value



        try:
            GPIO.setmode(GPIO.BOARD)  #  num of pin
            
            self.right_motor_pwm = R_PWM
            self.right_motor_in1 = R_IN1
            self.right_motor_in2 = R_IN2
            self.left_motor_pwm = L_PWM
            self.left_motor_in1 = L_IN1
            self.left_motor_in2 = L_IN2
            
            #  pines PWM
            GPIO.setup(self.right_motor_pwm, GPIO.OUT)
            GPIO.setup(self.right_motor_in1, GPIO.OUT)
            GPIO.setup(self.right_motor_in2, GPIO.OUT)
            GPIO.setup(self.left_motor_pwm, GPIO.OUT)
            GPIO.setup(self.left_motor_in1, GPIO.OUT)
            GPIO.setup(self.left_motor_in2, GPIO.OUT)
            
            self.right_pwm_device = GPIO.PWM(self.right_motor_pwm, 1000)
            self.left_pwm_device = GPIO.PWM(self.left_motor_pwm, 1000)
            self.right_pwm_device.start(0)  #  PWM a 0
            self.left_pwm_device.start(0)   #  PWM a 0

        except Exception as e:
            self.get_logger().error(f"Error al inicializar GPIO: {e}")
            raise

        # Sus /cmd_vel 
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info(f' \t Suscrito a /cmd_vel iniciada')

        # timer
        self._last_cmd_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.check_command_timeout)


    def check_command_timeout(self):

        if time.time() - self._last_cmd_time > 0.5:
            self.stop_motors()

    def cmd_vel_callback(self, msg: Twist):

        self._last_cmd_time = time.time()

        vx = msg.linear.x
        wz = msg.angular.z

        # limites max
        vx = max(min(vx, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        wz = max(min(wz, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)

        v_L = vx - wz 
        v_R = vx + wz
        
        if self.MAX_LINEAR_SPEED > 0.01:
            power_L = v_L / self.MAX_LINEAR_SPEED
            power_R = v_R / self.MAX_LINEAR_SPEED
        else:
            power_L = 0.0
            power_R = 0.0
            
        power_L = max(min(power_L, 1.0), -1.0)
        power_R = max(min(power_R, 1.0), -1.0)

        # isn`t necesary
        if self.INVERT_LEFT:
            power_L = -power_L
        if self.INVERT_RIGHT:
            power_R = -power_R

        # dirección y PWM
        self.set_motor_speed(self.left_pwm_device, self.left_motor_in1, self.left_motor_in2, power_L)
        self.set_motor_speed(self.right_pwm_device, self.right_motor_in1, self.right_motor_in2, power_R)

    def set_motor_speed(self, pwm_device, in1_pin, in2_pin, power: float):

        abs_power = abs(power)
        pwm_device.ChangeDutyCycle(abs_power * 100)  
        
        #  dirección 
        if power > 0:
            # adelante 
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        elif power < 0:
            # atras 
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
        else:
            # stop
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.LOW)

    def stop_motors(self):

        self.set_motor_speed(self.left_pwm_device, self.left_motor_in1, self.left_motor_in2, 0.0)
        self.set_motor_speed(self.right_pwm_device, self.right_motor_in1, self.right_motor_in2, 0.0)

    def on_shutdown(self):

        self.stop_motors()
        self.left_pwm_device.stop()
        self.right_pwm_device.stop()
        GPIO.cleanup()  # Limpiar pin
        self.get_logger().info('Limpiando GPIO')

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por usuario.')
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
