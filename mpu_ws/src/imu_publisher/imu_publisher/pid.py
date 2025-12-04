#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO

class DualMotorPID(Node):
    def __init__(self):
        super().__init__('dual_motor_pid_node')


        self.wheel_base = 0.19      
        self.wheel_diameter = 0.067  
        self.cpr_left = 972.0        
        self.cpr_right = 964.0       

 
        self.wheel_circ = math.pi * self.wheel_diameter

        # Encoder izquierdo
        self.L_ENC_A = 11  
        self.L_ENC_B = 13 

        # Encoder derecho
        self.R_ENC_A = 15  
        self.R_ENC_B = 16 

        # Motor izquierdo
        self.left_pwm_pin = 33 
        self.left_in1_pin = 37  
        self.left_in2_pin = 35  

        # Motor derecho
        self.right_pwm_pin = 32  
        self.right_in1_pin = 36
        self.right_in2_pin = 38


        GPIO.setmode(GPIO.BOARD)  # board
        
        #  encoder pull-up
        GPIO.setup(self.L_ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.L_ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.R_ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.R_ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(self.left_pwm_pin, GPIO.OUT)
        GPIO.setup(self.right_pwm_pin, GPIO.OUT)

        GPIO.setup(self.left_in1_pin, GPIO.OUT)
        GPIO.setup(self.left_in2_pin, GPIO.OUT)
        GPIO.setup(self.right_in1_pin, GPIO.OUT)
        GPIO.setup(self.right_in2_pin, GPIO.OUT)


        self.motor_left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        self.motor_right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        self.motor_left_pwm.start(0)
        self.motor_right_pwm.start(0)

        #  Variables PID 
        self.sp_l = 0.0   
        self.sp_r = 0.0  

        # Ganancias 
        self.kp = 1.5  
        self.ki = 0.2  
        self.kd = 0.0

        self.err_l = 0.0
        self.err_r = 0.0
        self.int_l = 0.0
        self.int_r = 0.0
        self.der_l = 0.0
        self.der_r = 0.0
        self.prev_err_l = 0.0
        self.prev_err_r = 0.0

        self.int_max = 50.0

        self.prev_left_steps = 0
        self.prev_right_steps = 0

        self.prev_time = self.get_clock().now()

        # pub - sub  
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.pub_rpm_left = self.create_publisher(Int32, 'rpm_left', 10)
        self.pub_rpm_right = self.create_publisher(Int32, 'rpm_right', 10)

        # Timer PID = 100 ms
        self.loop_dt = 0.01
        self.timer = self.create_timer(self.loop_dt, self.pid_loop)

        GPIO.add_event_detect(self.L_ENC_A, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.R_ENC_A, GPIO.RISING, callback=self.right_encoder_callback)


    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x    
        w = msg.angular.z   

        v_l = v - (self.wheel_base / 2.0) * w
        v_r = v + (self.wheel_base / 2.0) * w

        self.sp_l = (v_l * 60.0) / self.wheel_circ
        self.sp_r = (v_r * 60.0) / self.wheel_circ

    #  PID Loop 
    def pid_loop(self):
    
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.loop_dt
        self.prev_time = now

        steps_left = self.prev_left_steps
        steps_right = self.prev_right_steps

        delta_l = steps_left - self.prev_left_steps
        delta_r = steps_right - self.prev_right_steps
        self.prev_left_steps = steps_left
        self.prev_right_steps = steps_right

        rpm_l = 0.0
        rpm_r = 0.0
        if dt > 0:
            rpm_l = (delta_l / self.cpr_left) / dt * 60.0
            rpm_r = (delta_r / self.cpr_right) / dt * 60.0

        # VEL LINEAL
        circ = self.wheel_circ
        v_l_real = (rpm_l * circ) / 60.0
        v_r_real = (rpm_r * circ) / 60.0

    
        v_l_set = (self.sp_l * circ) / 60.0
        v_r_set = (self.sp_r * circ) / 60.0

        # PID IZQUIERDO 
        self.err_l = self.sp_l - rpm_l
        self.int_l += self.err_l * dt
        if self.int_l > self.int_max:
            self.int_l = self.int_max
        elif self.int_l < -self.int_max:
            self.int_l = -self.int_max

        if dt > 0:
            self.der_l = (self.err_l - self.prev_err_l) / dt
        else:
            self.der_l = 0.0

        out_l = self.kp * self.err_l + self.ki * self.int_l + self.kd * self.der_l
        self.prev_err_l = self.err_l

        # PID DERECHO 
        self.err_r = self.sp_r - rpm_r
        self.int_r += self.err_r * dt
        if self.int_r > self.int_max:
            self.int_r = self.int_max
        elif self.int_r < -self.int_max:
            self.int_r = -self.int_max

        if dt > 0:
            self.der_r = (self.err_r - self.prev_err_r) / dt
        else:
            self.der_r = 0.0

        out_r = self.kp * self.err_r + self.ki * self.int_r + self.kd * self.der_r
        self.prev_err_r = self.err_r

        # CONV to PWM 
        pwm_l = out_l / 255.0
        pwm_r = out_r / 255.0

        #  max
        pwm_l = max(0.0, min(1.0, pwm_l))
        pwm_r = max(0.0, min(1.0, pwm_r))

        if abs(v_l_set) < 0.01 and abs(v_r_set) < 0.01:
            self.stop_motors()
        else:
            self.set_motor_left(pwm_l)
            self.set_motor_right(pwm_r)

        # rpm
        msg_l = Int32()
        msg_r = Int32()
        msg_l.data = int(rpm_l)
        msg_r.data = int(rpm_r)
        self.pub_rpm_left.publish(msg_l)
        self.pub_rpm_right.publish(msg_r)


    def set_motor_left(self, pwm_value: float):
        self.motor_left_in1.on()
        self.motor_left_in2.off()
        self.motor_left_pwm.ChangeDutyCycle(pwm_value * 100)

    def set_motor_right(self, pwm_value: float):
        self.motor_right_in1.on()
        self.motor_right_in2.off()
        self.motor_right_pwm.ChangeDutyCycle(pwm_value * 100)

    def stop_motors(self):
        self.motor_left_pwm.ChangeDutyCycle(0.0)
        self.motor_right_pwm.ChangeDutyCycle(0.0)
        self.motor_left_in1.off()
        self.motor_left_in2.off()
        self.motor_right_in1.off()
        self.motor_right_in2.off()

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
