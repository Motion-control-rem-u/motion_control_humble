#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray

# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico cmd_Vel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String. 

# Este codigo es para ROS2.

# Basado en el trabajo hecho por  el subsistema motion control ROBOCOl Colombia,
# Implementación por Robocol en ROS1:https://github.com/Motion-control-rem-u/motion_control_rem_u.git 

class serialRaspESP(Node):

    def _init_(self):
        
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0, write_timeout=1)
        self.ser.reset_input_buffer()   
        print("Conexion Serial exitosa")     
        
        super()._init_('Serial_writer')
        self.subscription = self.create_subscription(Float32MultiArray, 'joystick', self.listener_callback, 10)
        self.left_f = 0 # Fuerza motor izq frontal
        self.right_f = 0 # fuerza motor der frontal
        self.left_b = 0 # Fuerza motor izq back
        self.right_b = 0 # fuerza motor der back
        

    def listener_callback(self, msg):
        self.left_f = int(msg.data[0])
        self.right_f = int (msg.data[1])
        self.left_b = int(msg.data[2])
        self.right_b = int (msg.data[3])
        pwms = [agregar_ceros(self.left_f), agregar_ceros(self.right_f),agregar_ceros(self.left_b), agregar_ceros(self.right_b)]          
        print (pwms)
        self.ser.write((str(pwms) + '\n').encode('utf-8'))


def agregar_ceros( self, numero):
    es_positivo=numero>=0
    numero_str=str(abs(numero))
    if es_positivo:
        numero_str="0"*(4-len(numero_str))+numero_str
    else:
        numero_str="-"+"0"*(3-len(numero_str))+numero_str
    return numero_str


def main(args=None):
    print("prueba 1")
    rclpy.init(args=args)
    SerialRaspESP=serialRaspESP()
    rclpy.spin(SerialRaspESP)
    SerialRaspESP.destroy_node()
    rclpy.shutdown()


if _name_ == '_main_':
    main()