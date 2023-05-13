import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray

import pygame

class Joystick_Publisher(Node):
    # Referenc de eventos generados de eje Movido:
    _axis_moved = False
    # Maximas rpm de las llantas
    _max_rpm = 250
    # Deadzone de la rotacion del joystick
    _deadzone_rot = 0.30
    #Deadzone del moviemiento del joystick
    _deadzone_stick = 0.1
    debug = input("Quiere debuggear? (1 o 0): ")
    if debug == "1":
        m1 = int(input("m1: "))
        m2 = int(input("m2: "))
        m3 = int(input("m3: "))
        m4 = int(input("m4: "))
    else:
        m1,m2,m3,m4 = (1,1,1,1)
    _enable = (m1,m2,m3,m4)

    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick', 10)
        timer_period = 0.1  # seconds

        pygame.init()
        
        pygame.joystick.init()

        print('Esperando joystick...')
        # Se crea la referencia al joystick
        joystick_ref = pygame.joystick.Joystick(0)
        # Se inicializa el joystick de esa referencia
        joystick_ref.init()
        # Referencia de la palanca estado actual
        ref_try_axis_3 = joystick_ref.get_axis(3)
        # Esperar a que se mueva palanca aceleracion para que se calibre
        while ref_try_axis_3 == joystick_ref.get_axis(3):  
            #rate.sleep()
            pygame.event.clear()
            print('Mover palanca por favor')
            pass  # Se corre el nodo hasta que este se finalice

        print('Joystick calibrado')
        msg = Float32MultiArray()
        self.timer = self.create_timer(timer_period, lambda : self.timer_callback(msg=msg, joystick_ref=joystick_ref))
    
    def timer_callback(self, msg, joystick_ref):

        msg.data = [0.0, 0.0, 0.0, 0.0]

        self.empty_event_queue()

        if self._axis_moved:
            axis0 = joystick_ref.get_axis(0)
            axis1 = joystick_ref.get_axis(1)
            axis2 = joystick_ref.get_axis(2)
            axis3 = joystick_ref.get_axis(3)
            
            if abs(round(axis2,1)) <= self._deadzone_rot:
                front_right,back_right,back_left,front_left = self.stick_steering(axis1, axis0, self._max_rpm*(-axis3+1)/2)
            else:
                front_right =  int(-(self._max_rpm*(-axis3+1)/2)*axis2)
                back_right = int(-(self._max_rpm*(-axis3+1)/2)*axis2)
                back_left = int((self._max_rpm*(-axis3+1)/2)*axis2)
                front_left = int((self._max_rpm*(-axis3+1)/2)*axis2)

            #msg.data[0],msg.data[1],msg.data[2], msg.data[3] = left_u,right_u,left_d,right_d
            msg.data[0],msg.data[1],msg.data[2], msg.data[3] = front_right * self._enable[0],-back_right * self._enable[1],-back_left * self._enable[2],front_left * self._enable[3]
            #encoded = (str(msg)+"\n").encode('utf-8')
            print(msg.data)
            self._axis_moved = False

            self.publisher_.publish(msg)
    
    def stick_steering(self, x, y, sensibilidad_rcv):
        # Convierte a polar
        r = math.hypot(-x, -y)
        t = math.atan2(-y, -x)
        # Rota 45 grados
        t += math.pi / 4
        # Retorna a cartesianas
        left_u = r * math.cos(t)
        right_u = r * math.sin(t)
        left_d = r * math.cos(t)
        right_d = r * math.sin(t)
        # Reescala a nuevas coordenadas
        left_u = left_u * math.sqrt(2)
        right_u = right_u * math.sqrt(2)
        left_d = left_d * math.sqrt(2)
        right_d = right_d * math.sqrt(2)
        # clamp to -1/+1
        left_u = max(min(left_u, 1), -1)
        right_u = max(min(right_u, 1), -1)
        left_d = max(min(left_d, 1), -1)
        right_d = max(min(right_d, 1), -1)

        min_mov = self._deadzone_stick*math.sqrt(2)

        if(r<=min_mov):
            left_u,right_u,left_d,right_d = 0,0,0,0

        return int(sensibilidad_rcv*left_u), int(sensibilidad_rcv*right_u),int(sensibilidad_rcv*left_d), int(sensibilidad_rcv*right_d)

    def empty_event_queue(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self._axis_moved = True
            elif event.type == pygame.JOYBALLMOTION:
                pass
            elif event.type == pygame.JOYHATMOTION:
                pass
            elif event.type == pygame.JOYBUTTONDOWN:
                pass
            elif event.type == pygame.JOYBUTTONUP:
                pass



def main():
    rclpy.init()
    joystick_publisher = Joystick_Publisher()
    rclpy.spin(joystick_publisher)

    joystick_publisher.destroy_node()
    rclpy.shutdown()

# if __name__ == '__main__':
#     main()
