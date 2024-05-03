#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerilReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")
        self.pub_=self.create_publisher(String,"Serial_receiver",10)
        self.freequency = 0.01
        self.get_logger().info("publishing at %d Hz" %self.freequency)
        self.declare_parameter("port","/dev/ttyUSB0")
        self.declare_parameter("baud_rate","115200")
        self.port_=self.get_parameter("port").value
        self.baudrate_=self.get_parameter("baud_rate").value
        self.arduino_=serial.Serial(self.port_,self.baudrate_)
        self.timer=self.create_timer(0.1,self.timer_callback)


    def timer_callback(self):
        if rclpy.ok() & self.arduino_.is_open:
            data=self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return
            msg=String()
            msg.data=str(data)
            self.pub_.publish(msg)
    
def main():
    rclpy.init()
    srvr=SimpleSerilReceiver()
    rclpy.spin(srvr)
    srvr.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

