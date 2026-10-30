#!/usr/bin/env python3
"""Debug aid: print the closest lidar returns (range, bearing) of one /scan.

  tools/ros python3 /home/ubuntu/tools/scan_min.py
"""
import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('scan_min')
    received = []
    node.create_subscription(LaserScan, '/scan', received.append, qos_profile_sensor_data)
    while not received:
        rclpy.spin_once(node, timeout_sec=1.0)
    scan = received[0]
    returns = sorted(
        (distance, math.degrees(scan.angle_min + index * scan.angle_increment))
        for index, distance in enumerate(scan.ranges) if math.isfinite(distance))
    print(f'frame {scan.header.frame_id}, range_min {scan.range_min}, '
          f'{len(returns)} finite returns')
    for distance, bearing_deg in returns[:10]:
        print(f'  {distance:.3f} m at {bearing_deg:7.1f} deg')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
