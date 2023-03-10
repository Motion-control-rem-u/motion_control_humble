import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import serial
import serial.tools.list_ports


class SerialWriter(Node):
    '''
    OBTIENE PWM DE CONTROL AUTONOMO O DE MANUAL Y LO ESCRIBE POR SERIAL.
    Corre en la Jetson
    funciona con joystick _publisher.py y control_rem.py
    ROBOCOL 2023-1
    '''

    def __init__():
        super.__init__('node_serial_writer')

    pass