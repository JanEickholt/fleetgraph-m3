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
    got = []
    node.create_subscription(LaserScan, '/scan', got.append, qos_profile_sensor_data)
    while not got:
        rclpy.spin_once(node, timeout_sec=1.0)
    s = got[0]
    pts = sorted((r, math.degrees(s.angle_min + i * s.angle_increment))
                 for i, r in enumerate(s.ranges) if math.isfinite(r))
    print(f'frame {s.header.frame_id}, range_min {s.range_min}, {len(pts)} finite returns')
    for r, a in pts[:10]:
        print(f'  {r:.3f} m at {a:7.1f} deg')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
