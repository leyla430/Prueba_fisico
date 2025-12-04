#!/usr/bin/env python3


# //////////////////////////// SI ///////////////////////////


import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32

class RobotHeightScan(Node):
    def __init__(self):
        super().__init__('robot_height_scan')

        # ---------- Parámetros ----------
        self.scan_frame = self.declare_parameter('scan_frame', 'base_scan') \
                            .get_parameter_value().string_value
        self.input_cloud   = self.declare_parameter('input_cloud', '/points2').get_parameter_value().string_value
        self.scan_topic    = self.declare_parameter('scan_topic',  '/scan').get_parameter_value().string_value

        self.tilt_deg      = float(self.declare_parameter('tilt_deg', 90.0).value)
        self.pitch_min_deg = float(self.declare_parameter('pitch_min_deg',  2.0).value)
        self.pitch_max_deg = float(self.declare_parameter('pitch_max_deg', 17.0).value)
        self.yaw_min_deg   = float(self.declare_parameter('yaw_min_deg', -150.0).value)
        self.yaw_max_deg   = float(self.declare_parameter('yaw_max_deg',  150.0).value)
        self.bin_width_deg = float(self.declare_parameter('bin_width_deg', 2.0).value)
        self.range_min = float(self.declare_parameter('range_min', 0.10).value)
        self.range_max = float(self.declare_parameter('range_max', 30.0).value)
        self.percentile = float(self.declare_parameter('percentile', 0.15).value)
        self.stride     = int(self.declare_parameter('stride', 1).value)
        self.front_sector_deg = float(self.declare_parameter('front_sector_deg', 180.0).value)
        self.use_horizontal_range = bool(self.declare_parameter('use_horizontal_range', True).value)

        # ---------- Pre-cálculos ----------
        self.tilt_rad    = math.radians(self.tilt_deg)
        self.ct, self.st = math.cos(-self.tilt_rad), math.sin(-self.tilt_rad)
        self.pitch_min = math.radians(self.pitch_min_deg)
        self.pitch_max = math.radians(self.pitch_max_deg)
        self.yaw_min   = math.radians(self.yaw_min_deg)
        self.yaw_max   = math.radians(self.yaw_max_deg)
        self.bin_w     = math.radians(self.bin_width_deg)
        span = max(1e-6, self.yaw_max - self.yaw_min)
        self.num_bins = int(math.floor(span / self.bin_w)) + 1

        # ---------- QoS: usa SensorData (BestEffort) ----------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(PointCloud2, self.input_cloud, self.cb, sensor_qos)
        self.pub_scan = self.create_publisher(LaserScan, self.scan_topic, sensor_qos)
        self.pub_front = self.create_publisher(Float32, '/obstacle_distance_front', sensor_qos)

        self.get_logger().info(
            f"PointCloud2 ({self.input_cloud}) -> LaserScan ({self.scan_topic}) | "
            f"QoS=SensorData(BestEffort) | frame={self.scan_frame}"
        )

    def cb(self, msg: PointCloud2):
        bins = [[] for _ in range(self.num_bins)]
        try:
            points = pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
            for i, (x, y, z) in enumerate(points):
                if self.stride > 1 and (i % self.stride) != 0:
                    continue
                r3 = math.sqrt(x*x + y*y + z*z)
                if r3 < self.range_min or r3 > self.range_max:
                    continue

                # “des-rotar” nube (si tu LIDAR está echado)
                xp = x
                yp =  y * self.ct + z * self.st
                zp = -y * self.st + z * self.ct

                yaw   = math.atan2(yp, xp)
                horiz = math.hypot(xp, yp)
                pitch = math.atan2(zp, horiz)

                if not (self.pitch_min <= pitch <= self.pitch_max):
                    continue
                if not (self.yaw_min <= yaw <= self.yaw_max):
                    continue
                if horiz < self.range_min or horiz > self.range_max:
                    continue

                idx = int((yaw - self.yaw_min) / self.bin_w)
                if 0 <= idx < self.num_bins:
                    bins[idx].append(horiz)

        except Exception as e:
            self.get_logger().warn(f"Error processing point cloud: {e}")
            return

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.yaw_min
        scan.angle_max = self.yaw_max
        scan.angle_increment = self.bin_w
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        ranges = [float('inf')] * self.num_bins
        for i in range(self.num_bins):
            if bins[i]:
                ranges[i] = float(min(bins[i]))  # mínimo = “primera colisión”
        scan.ranges = ranges

        self.pub_scan.publish(scan)

        # dist. mínima al frente (±front_sector/2)
        half = math.radians(self.front_sector_deg / 2.0)
        front_min = float('inf')
        for i in range(self.num_bins):
            ang = self.yaw_min + i * self.bin_w
            if -half <= ang <= half and not math.isinf(scan.ranges[i]):
                front_min = min(front_min, scan.ranges[i])
        self.pub_front.publish(Float32(data=front_min))

def main():
    rclpy.init()
    node = RobotHeightScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
