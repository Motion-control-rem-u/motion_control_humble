import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Bool

import serial
import serial.tools.list_ports
from serial.serialutil import SerialException

import threading

class SerialWriter(Node):
    '''
    OBTIENE PWM DE CONTROL AUTONOMO O DE MANUAL Y LO ESCRIBE POR SERIAL.
    Corre en la Jetson
    funciona con joystick_publisher.py y control_rem.py
    ROBOCOL 2023-1
    '''
    _flag_autonomo = False

    _vel_izq_u = 0
    _vel_der_u = 0
    _vel_izq_d = 0
    _vel_der_d = 0

    #Modo para funcionamiento drivers (d=drive, b=break, r=reverse)
    _modo = 'd'

    def __init__(self):
        super().__init__('node_serial_writer')

        arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=10)

        self.timer = threading.Timer(2.0, self.detener)
        self.test = True
        self.pwm_data_subscriber = self.create_subscription(
            Float32MultiArray,
            '/joystick',
            lambda msg : self.pwm_data_callback(msg=msg, arduino=arduino),
            10
        )

        self.flag_autonomo_subscriber = self.create_subscription(
            Bool,
            'motion/flag_autonomo',
            self.flag_autonomo_callback,
            10
        )
        print("Nodo Iniciado")

    def pwm_data_callback(self, msg, arduino):
        #TODO editar cuando tengamos nav autonomo
        if self.test:
            self.test = False
            self.timer.start()
        self.timer.cancel()
        self.timer = threading.Timer(2.0, self.detener)
        order = [0,0,0,0, self._modo]
        self._vel_izq_u = int(msg.data[0])
        self._vel_der_u = int(msg.data[1])
        self._vel_izq_d = int(msg.data[2])
        self._vel_der_d = int(msg.data[3])

        left_u =  self.agregar_ceros(self._vel_izq_u)
        right_u = self.agregar_ceros( self._vel_der_u)
        left_d =  self.agregar_ceros( self._vel_izq_d)
        right_d = self.agregar_ceros( self._vel_der_d)

        (order[0], order[1],order[2],order[3]) =  ( left_u,right_u,left_d,right_d)
        encoded = (str(order) + '\n').encode('utf-8')
        #print(encoded)

        self.arduino = arduino
        arduino.write(encoded)
        print(arduino.readline())
        self.timer.start()

    def flag_autonomo_callback(self, msg):
        #TODO
        self._flag_autonomo = bool(msg.data)
        self._flag_autonomo = False
    # Agregar ceros (para que se mande por serial bien)
    def agregar_ceros(self,numero):
        es_positivo=float(numero)>=0
        numero_str=str(abs(numero))
        if es_positivo:
            numero_str="0"*(4-len(numero_str))+numero_str
        else:
            numero_str="-"+"0"*(3-len(numero_str))+numero_str
        return numero_str
    def detener(self):
        print("Codigo antisuicida activo!!")
        order = [0,0,0,0, self._modo]
        encoded = (str(order) + '\n').encode('utf-8')
        self.arduino.write(encoded)



def main():
    rclpy.init()
    try:
        serial_writer = SerialWriter()
        try:
            rclpy.spin(serial_writer)
        except KeyboardInterrupt:
            sys.stdout.write("\033[F")
            print('Ctrl-c detectado. Chao.')
        finally:
            serial_writer.destroy_node()
    except SerialException:
        print("Error serial. Esta conectado el arduino? Intenta de nuevo.")
