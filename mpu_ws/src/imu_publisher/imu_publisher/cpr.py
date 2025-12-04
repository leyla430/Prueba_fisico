#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import Jetson.GPIO as GPIO

# Pines de encoders 
L_ENC_A = 11
L_ENC_B = 13
R_ENC_A = 15
R_ENC_B = 16

class CPRCalibrator(Node):
    def __init__(self):
        super().__init__('cpr_calibrator')
        self.get_logger().info('Iniciando calibración de CPR (Modo x4)...')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(L_ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(L_ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(R_ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(R_ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # contadores
        self.left_ticks = 0
        self.right_ticks = 0

        # interrupciones 
        GPIO.add_event_detect(L_ENC_A, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(R_ENC_A, GPIO.RISING, callback=self.right_encoder_callback)

        time.sleep(1)

    def calibrate(self):

        time.sleep(1.0) 

        self.get_logger().info("\n--- Paso 1: Calibración Motor IZQUIERDO ---")
        self._measure_motor(self.left_ticks, "IZQUIERDO")

        self.get_logger().info("\n--- Paso 2: Calibración Motor DERECHO ---")
        self._measure_motor(self.right_ticks, "DERECHO")

        self.get_logger().info("\n--- Calibración Finalizada ---")
        self.get_logger().info("Ahora puedes usar estos valores en tu código PID.")
        time.sleep(1.0) 

    def _measure_motor(self, ticks, name):
        """Mide el CPR para un motor dado."""

        self.left_ticks = 0 if name == "IZQUIERDO" else self.left_ticks
        self.right_ticks = 0 if name == "DERECHO" else self.right_ticks

        self.get_logger().info(f"Rueda {name}:")
        self.get_logger().warn(f"-> Gira la rueda {name} **EXACTAMENTE UNA VUELTA COMPLETA** (360°).")
        input("-> Presiona ENTER cuando estés listo para comenzar...")
        
        start_count = self.left_ticks if name == "IZQUIERDO" else self.right_ticks
        self.get_logger().info(f"Conteo inicial: {start_count}")
        
        input("-> Presiona ENTER cuando hayas completado la vuelta (360°)...")
        
        final_count = self.left_ticks if name == "IZQUIERDO" else self.right_ticks
        cpr_x4 = abs(final_count - start_count)
        

        self.get_logger().info(f" CPR Real (x4) para el Motor {name}: {cpr_x4} pulsos/vuelta")


    def left_encoder_callback(self, channel):
        """Cuenta los ticks del encoder izquierdo."""
        self.left_ticks += 1

    def right_encoder_callback(self, channel):
        """Cuenta los ticks del encoder derecho."""
        self.right_ticks += 1

    def on_shutdown(self):
        """Limpieza de GPIO al cerrar el nodo."""
        GPIO.cleanup()  # Limpiar los pines al finalizar
        self.get_logger().info('Apagando y limpiando GPIO...')


def main(args=None):
    rclpy.init(args=args)
    calibrator = CPRCalibrator()
    try:
        calibrator.calibrate()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
