#!/usr/bin/env python3

import rclpy
from sensors_pkg.stepper_encoder import AS5600

def main(args = None):
    rclpy.init(args=args)
    Encoders_node = AS5600()
    rclpy.spin(Encoders_node)
    rclpy.shutdown()

if __name__ ==  "__main__":
    main()