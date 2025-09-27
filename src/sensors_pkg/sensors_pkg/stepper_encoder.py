#!/usr/bin/env python3
from smbus2 import SMBus
import smbus2
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Encoders
from sensor_msgs.msg import JointState
import time

class AS5600(Node):
    def __init__(self):
        super().__init__("AS5600_node")
        self.pub = self.create_publisher(JointState,'joint_states',10)
        self.bus = smbus2.SMBus(1)
        self.timer = self.create_timer(10.00/1000.00, self.timer_callback)
        self.joints_angle = [0,0,0,0,0,0,0]
        self.declare_parameter("mux_adress",0x70)
        self.fin = 0
        self.inicio = 0


    def read_angle(self):
        mux_address = self.get_parameter("mux_adress").get_parameter_value().integer_value
        self.inicio = time.time()

        #Read two bytes from the angle register
        #self.fin = time.time()
        for channel in range(8):
        
        # 1. Select the channel on the multiplexer
        # The PCA9548A is controlled by writing a single byte. Each bit corresponds
        # to a channel. `1 << channel` creates a byte with only the bit for the
        # current channel set (e.g., channel 0 is 0b00000001, channel 3 is 0b00001000).
            try:
                self.bus.write_byte(mux_address, 1 << channel)
                error = False
            except OSError as e:
                error = True
                # Exit the function if the multiplexer isn't accessible

        # 2. Scan the I2C bus for devices on the selected channel
        # We scan the standard I2C address range (0x08 to 0x77).
            if not error:
                try:
                    raw_data = self.bus.read_i2c_block_data(0x36, 0x0C, 2)
                    angle = (raw_data[0] << 8) | raw_data[1]  # Combine MSB and LSB
                    angle = angle & 0x0FFF  # Mask to 12 bits   
                    self.joints_angle[channel] = (angle / 4096.0) * 360.0 
                except OSError as e:
                    error = True
    
    def timer_callback(self):
        msg = JointState()
        #self.inicio = time.time()
        msg.name = ["joint0","joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
        self.read_angle()
        msg.position = self.joints_angle    
        self.pub.publish(msg)
        #self.fin = time.time()
        self.get_logger().info('Publishing: {0}' .format(msg.position))
        #self.get_logger().info('I2C reading time: {0}' .format(self.fin - self.inicio))
    
