#!/usr/bin/env python3
from smbus2 import SMBus
import smbus2
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Encoders
import time

class AS5600(Node):
    def __init__(self):
        super().__init__("AS5600_node")
        self.pub = self.create_publisher(Encoders,'stepper_pos',10)
        self.bus = smbus2.SMBus(1)
        self.timer = self.create_timer(10.00/1000.00, self.timer_callback)
        # self.fin = 0
        # self.inicio = 0


    def read_angle(self):
        #self.inicio = time.time()
        #Read two bytes from the angle register
        raw_data = self.bus.read_i2c_block_data(0x36, 0x0C, 2)
        angle = (raw_data[0] << 8) | raw_data[1]  # Combine MSB and LSB
        angle = angle & 0x0FFF  # Mask to 12 bits
        #self.fin = time.time()
        return (angle / 4096.0) * 360.0  # Convert to degrees
    
    def timer_callback(self):
        msg = Encoders()
        msg.position[0] = self.read_angle()
        self.pub.publish(msg)
        self.get_logger().info('Publishing: {0}' .format(msg.position))
        #self.get_logger().info('I2C reading time: {0}' .format(self.fin - self.inicio))
    
