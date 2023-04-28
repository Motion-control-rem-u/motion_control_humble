#!/usr/bin/env python3 
import rclpy
import time
#import serial
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray, Bool
import time

# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico cmd_Vel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String. 

# Este codigo es para ROS2.

# Basado en el trabajo hecho por  el subsistema motion control ROBOCOl Colombia,
# Implementación por Robocol en ROS1:https://github.com/Motion-control-rem-u/motion_control_rem_u.git 

class serialRaspESP(Node):

    def __init__(self):
        try:
            #self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.ser.reset_input_buffer()   
            print("Conexion Serial exitosa")     
        except Exception:
            pass
        super().__init__('move_forward')
        self.subscription = self.create_subscription(Bool, 'move_forward_flag', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick', 10)
        timer_period = 0.01  # seconds
        
        #self.timer = self.create_timer(timer_period, lambda : self.timer_callback(msg=msg))
    

        self.left_f = 0 # Fuerza motor izq frontal
        self.right_f = 0 # fuerza motor der frontal
        self.left_b = 0 # Fuerza motor izq back
        self.right_b = 0 # fuerza motor der back
        

    def agregar_ceros(numero):
        es_positivo=numero>=0
        numero_str=str(abs(numero))
        if es_positivo:
            numero_str="0"*(4-len(numero_str))+numero_str
        else:
            numero_str="-"+"0"*(3-len(numero_str))+numero_str
        return numero_str

    def listener_callback(self, msg):
        
        
        self.left_f = int(180)
        self.right_f = int (180)
        self.left_b = int(180)
        self.right_b = int (180)
        inicio = float(time.time())
        duracion = float(input("Duración del move_forward (nS): "))
            
        while ((time.time() - inicio ) < duracion ) and (msg.data == True) :
            pwms = Float32MultiArray()
            pwms.data = [0, 0, 0, 0]
            #pwms = [(self.left_f), (self.right_f),(self.left_b), (self.right_b)]          
            pwms.data[0],pwms.data[1],pwms.data[2], pwms.data[3]  = self.left_f,self.right_f,self.left_b,self.right_b
        
            print (pwms.data)
            print ((time.time() - inicio ))
            self.publisher_.publish(pwms)
        pwms.data = [0, 0, 0, 0]
        print (pwms.data)
        self.publisher_.publish(pwms)

def main(args=None):
    rclpy.init(args=args)
    SerialRaspESP=serialRaspESP()
    rclpy.spin(SerialRaspESP)
    SerialRaspESP.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()