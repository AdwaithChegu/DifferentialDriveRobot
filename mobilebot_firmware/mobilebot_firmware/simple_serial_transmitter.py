#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("Simple_Serial_Transmitter")

        self.declare_parameter("port","/dev/ttyUSB0")
        self.declare_parameter("baud_rate",115200)

        self.port_=self.get_parameter("port").value
        self.baud_rate=self.get_parameter("baud_rate").value

        self.sub_=self.create_subscription(String,"serial_transmitter",self.msgCallback,10)
        self.arduino_=serial.Serial(port=self.port_,baudrate=self.baud_rate,timeout=0.1)

        

    def msgCallback(self,msg):
        self.get_logger().info("New Message Received,publishing on serial : %s" % self.arduino_.name)
        self.arduino_.write(msg.data.encode("UTF-8"))

def main():
    rclpy.init()
    Simple_Serial_Transmitter=SimpleSerialTransmitter()
    rclpy.spin(Simple_Serial_Transmitter)
    Simple_Serial_Transmitter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
 