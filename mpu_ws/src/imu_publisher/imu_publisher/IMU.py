import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import smbus
import time
import numpy as np

# --- MPU9250 ---
BUS_NUM = 1                      # bus 1 detectado
ADDRESS = 0x68                   # dirección 
G_TO_MS2 = 9.80665              

PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
WHO_AM_I = 0x75
TEMP_OUT_H = 0x41
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D  
ACCEL_ZOUT_H = 0x3F  
GYRO_XOUT_H = 0x43


AFS_SEL = 0 
GFS_SEL = 0 

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Pub
        self.imu_pub = self.create_publisher(Imu, '/demo/imu', 10) 
        self.timer = self.create_timer(0.05, self.update)  
        self.imu_frame_id = "imu_link" 
        
        self.get_logger().info(f"Inicializando MPU9250 en Bus {BUS_NUM}")
        
        # escala
        self.ACCEL_SENSITIVITY = 16384.0 
        self.GYRO_SENSITIVITY = 131.0  
        
        # inicializar I2C
        self.bus = self.init_mpu()
        
        if self.bus is None:
            self.get_logger().error("IMU no inicializada, terminando el nodo.")
            


    def read_word_2c(self, adr):

        high = self.bus.read_byte_data(ADDRESS, adr)
        low = self.bus.read_byte_data(ADDRESS, adr + 1)
        val = (high << 8) + low
        
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def init_mpu(self):
        try:
            bus = smbus.SMBus(BUS_NUM)
            

            device_id = bus.read_byte_data(ADDRESS, WHO_AM_I)
            if device_id != 0x71:  
                self.get_logger().error(f"ID MPU incorrecto: 0x{device_id:x}. Esperado 0x71.")
                return None
            
            # configurar
            bus.write_byte_data(ADDRESS, PWR_MGMT_1, 0) 
            bus.write_byte_data(ADDRESS, GYRO_CONFIG, GFS_SEL << 3)
            bus.write_byte_data(ADDRESS, ACCEL_CONFIG, AFS_SEL << 3) 
            
            self.get_logger().info(f" MPU9250 configurado en Bus {BUS_NUM}.")
            return bus
            
        except Exception as e:
            self.get_logger().error(f'Fallo al inicializar MPU en Bus {BUS_NUM}: {e}')
            return None

    # Pub
    def update(self):
        if self.bus is None:
            return

        try:
            
            # Acelerómetro
            accel_x_raw = self.read_word_2c(ACCEL_XOUT_H)
            accel_y_raw = self.read_word_2c(ACCEL_YOUT_H) 
            accel_z_raw = self.read_word_2c(ACCEL_ZOUT_H) 
            
            # Giroscopio
            gyro_x_raw = self.read_word_2c(GYRO_XOUT_H)
            gyro_y_raw = self.read_word_2c(GYRO_XOUT_H + 2)
            gyro_z_raw = self.read_word_2c(GYRO_XOUT_H + 4)

            #  Escala y Conversión
            accel_x = (accel_x_raw / self.ACCEL_SENSITIVITY) * G_TO_MS2
            accel_y = (accel_y_raw / self.ACCEL_SENSITIVITY) * G_TO_MS2
            accel_z = (accel_z_raw / self.ACCEL_SENSITIVITY) * G_TO_MS2
            
            # vel Angular 
            gyro_x = np.deg2rad(gyro_x_raw / self.GYRO_SENSITIVITY)
            gyro_y = np.deg2rad(gyro_y_raw / self.GYRO_SENSITIVITY)
            gyro_z = np.deg2rad(gyro_z_raw / self.GYRO_SENSITIVITY)

            # msg imu
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame_id 

            imu_msg.orientation.w = 1.0 
            imu_msg.orientation_covariance[0] = -1.0
            
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.angular_velocity_covariance[0] = 0.02
            
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            imu_msg.linear_acceleration_covariance[0] = 0.1

            self.imu_pub.publish(imu_msg)

            self.get_logger().info(
                 f" IMU publicado: w_z: {imu_msg.angular_velocity.z:.2f}, acc_x: {imu_msg.linear_acceleration.x:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f'Error al leer o publicar MPU: {e}')
            
def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error fatal: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
