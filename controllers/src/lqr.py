#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from tum_ardrone.msg import filter_state
from sensor_msgs.msg import Imu
import math
import numpy as np
from std_msgs.msg import String
from controllers.srv import SetReference
from controllers.srv import CircParameters
from controllers.msg import poseref
from std_srvs.srv import Trigger, TriggerResponse
from keyboard.msg import Key


def angle_from_to_2(angle, min_angle, max_angle):
    while angle < min_angle:
        angle += 360
    while angle >= max_angle:
        angle -= 360
    return angle



class ControllerNode:

    def __init__(self):
        # Inicializa o node do ROS
        rospy.init_node('controller_node', anonymous=True)
        # Cria o objeto de publicação no tópico /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.poseref_pub = rospy.Publisher('/poseref', poseref, queue_size=10)
        # Cria os objetos de inscrição nos tópicos /pose e /imu
        rospy.Subscriber('/ardrone/predictedPose', filter_state, self.pose_callback)
        rospy.Subscriber('/ardrone/imu_throttle', Imu, self.imu_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.keyboard_callback)
        # Define a frequência de publicação
        self.rate = rospy.Rate(100) # 100 Hz,
        # Cria o serviço de definição de referência
        rospy.Service('setref', SetReference, self.set_reference)
        # Cria o serviço de trajetória circular
        rospy.Service('circ', CircParameters, self.set_circ)        
        # Cria o serviço de trajetória infinito
        self.command_service_inf = rospy.Service('inf', Trigger, self.set_inf)
        # Cria o serviço de trajetória quadrática
        self.command_service_quad = rospy.Service('quad', Trigger, self.set_quad)
        # Tempo inicial
        t = rospy.Time.now()
        # Variáveis de estado do sistema
        self.onoff = False
        self.t = rospy.Time.now()
        self.circ_radius = 0.8  #Radius of the circle
        self.circ_angular_velocity = 0.05  # angular velocity of the circle in rad/s
        self.circ_z = 0.7  #Height of the circle in meters
        self.pitch = 0.0
        self.dpitch = 0.0
        self.x = 0.0
        self.dx = 0.0
        self.roll = 0.0
        self.droll = 0.0
        self.y = 0.0
        self.dy = 0.0
        self.z = 0.0
        self.dz = 0.0
        self.yaw = 0.0    
        self.dyaw = 0.0
        self.Xstates = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.Posereal = [0,0,0,0]
        self.Poseref = [0,0,0,0]
        self.error = [0,0,0,0]
        self.error_integral = np.zeros(4)
        self.error_previous = np.zeros(4)
        self.error_Ki = np.zeros(4)
        self.u_control = np.zeros(4)
        self.ref_service_called = False
        self.circ_service_called = False
        self.inf_service_called = False
        self.quad_service_called = False
        self.step = 0
        self.count = 0
    # Função de serviço para definir a referência
    def set_reference(self, req):
        self.Poseref[0] = req.x
        self.Poseref[1] = req.y
        self.Poseref[2] = req.z
        self.Poseref[3] = angle_from_to_2(req.heading, -180, 180)*math.pi/180
         # Atribui True à variável quando o serviço é chamado
        self.ref_service_called = True
        self.circ_service_called = False
        self.inf_service_called = False
        self.quad_service_called = False
        print('Novo Target recebido')
        self.count = 0
        return True
    # Função de serviço para definir a trajetória circular
    def set_circ(self, req):
        self.circ_radius = req.radius  #Radius of the circle
        self.circ_angular_velocity = req.angular_velocity  # angular velocity of the circle in rad/s
        self.circ_z = req.height  #Height of the circle in meters
        self.circ_service_called = True
        self.ref_service_called = False
        self.inf_service_called = False
        self.quad_service_called = False
        print('Iniciando trajetória circular')
        self.count = 0
        return True
        # Função de serviço para definir a trajetória circular
    def set_inf(self, req):
        self.circ_service_called = False
        self.ref_service_called = False
        self.inf_service_called = True
        self.quad_service_called = False
        # Retorna a resposta do serviço
        response = TriggerResponse()
        response.success = True
        response.message = 'Iniciando trajetória de infinito'
        self.count = 0
        return response
    def set_quad(self, req):
        self.circ_service_called = False
        self.ref_service_called = False
        self.inf_service_called = False
        self.quad_service_called = True
        # Retorna a resposta do serviço
        response = TriggerResponse()
        response.success = True
        response.message = 'Iniciando trajetória quadrática'
        self.count = 0
        return response
    # Função de callback para o tópico /pose
    def pose_callback(self, msg):
        self.pitch = -msg.pitch*math.pi/180
        self.x = msg.y
        self.dx = msg.dy
        self.roll = -msg.roll*math.pi/180
        self.y = msg.x
        self.dy = msg.dx
        self.z = msg.z
        self.dz = msg.dz
        self.yaw = -angle_from_to_2(msg.yaw, -180, 180)*math.pi/180    
        self.dyaw = -msg.dyaw*math.pi/180
        #//y yponto x xponto -theta -thetaponto -rolll -rollponto z zponto -yaw -yawponto ex ey ez -eyaw
    # Função de callback para o tópico /imu
    def imu_callback(self, msg):
        self.dpitch = msg.angular_velocity.y*math.pi/180
        self.droll = -msg.angular_velocity.x*math.pi/180
    def keyboard_callback(self, data):
        global is_turned_on
        if data.code == 32:  # Código da tecla 'espaço'
            is_turned_on = not is_turned_on
            if is_turned_on:
                rospy.loginfo("Controlador ligado")
                self.onoff = True
            else:
                rospy.loginfo("Controlador desligado")
                self.onoff = False


    # Função principal de controle
    def run(self):
        while not rospy.is_shutdown():
            vel_cmd = Twist()
            pose_ref = poseref()
            while self.onoff:
                controller.Xstates = [controller.pitch, controller.dpitch, controller.x, controller.dx, controller.roll, controller.droll, controller.y, controller.dy, controller.z, controller.dz, controller.yaw, controller.dyaw]
                controller.Posereal = [controller.x, controller.y, controller.z, controller.yaw]                
                if self.ref_service_called:
                    self.count = 0
                elif self.circ_service_called:
                    # Calcula a posição atual do ponto no círculo
                    current_time = rospy.Time.now()
                    delta_t = (current_time - self.t).to_sec()
                    theta = self.circ_angular_velocity * delta_t
                    self.Poseref[0] = self.circ_radius*math.sin(theta)
                    self.Poseref[1] = self.circ_radius*math.cos(theta)
                    self.Poseref[2] = self.circ_z
                    self.Poseref[3] = 0 
                elif self.inf_service_called:
                    current_time = rospy.Time.now()
                    delta_t = (current_time - self.t).to_sec()
                    self.Poseref[0] = math.sin(0.05*delta_t)
                    self.Poseref[1] = 0.5*math.sin(0.1*delta_t)
                    self.Poseref[2] = 0.7 + 0.5*math.sin(0.05*delta_t)
                    self.Poseref[3] = -(math.pi/12)*math.sin(0.05*delta_t) 
                    self.count = 0
                elif self.quad_service_called: 
                    if self.count == 0:
                        self.Poseref = [0,0,0.4,0] 
                    elif self.count == 1:
                        self.Poseref = [0,0.5,0.4,0]
                    elif self.count == 2:
                        self.Poseref = [0,0.5,0.7,0]
                    elif self.count == 3:
                        self.Poseref = [0,0,0.7,0]
                    elif self.count == 4:
                        self.Poseref = [0,-0.5,0.7,0]
                    elif self.count == 5:
                        self.Poseref = [0,-0.5,0.4,0]  
                    elif self.count == 6:
                        self.Poseref = [0,0,0.4,0] 
                else:
                    self.Poseref = [0,0,0.4,0]
                    self.count = 0
                
                gainselect = "Rodrigo"
                # gainselect = "simples"
                # gainselect = "robusto"
                # gainselect = "teste"

                if gainselect == "Rodrigo":
                    K = [[-3.06, -0.46, -3.12, -1.59, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, -3.42, -0.48, 3.63, 1.64, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -5.15, -1.59, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.19, -0.08]] 
                    Ki= [[2.86, 0, 0, 0],
                            [0,-3.43, 0, 0],
                            [0, 0, 4.47,0],
                            [0, 0, 0, 1.0]] 
                elif gainselect == "simples":
                    K = [[-2.3300, -0.4578, -1.7240, -0.8035, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, -2.3833, -0.6595, 1.4823, 0.7344, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, -2.0816, -0.6692, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.6427, -0.4653]] 
                    Ki= [[0.7476, 0, 0, 0],
                         [0,-0.6028, 0, 0],
                         [0, 0, 1.0626,0],
                         [0, 0, 0, 0.1387]] 
                elif gainselect == "robusto":
                    K = [[-2.4496, -0.5186, -1.5088, -0.8481, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, -2.9098, -0.9564, 1.4914, 0.8853, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -1.0574, -0.3941, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.9184, -0.1469]] 
                    Ki= [[0.2587, 0, 0, 0],
                        [0,-0.2608, 0, 0],
                        [0, 0, 0.1973,0],
                        [0, 0, 0, 0.4797]]
                else:
                    K = [[-1.9147, -0.6052, -0.8325, -0.9071, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, -1.9251, -0.6058, 0.6746, 0.9130, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -2.0768, -0.6069, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5572, -0.0835]]
                    Ki= [[0.2274, 0, 0, 0],
                        [0,-0.1666, 0, 0],
                        [0, 0, 0.9764,0],
                        [0, 0, 0, 0.2774]] 
  
                Xstatesb = np.dot(K,controller.Xstates)
                self.error = np.subtract(controller.Poseref, controller.Posereal)

                if all(elemento > -0.05 for elemento in self.error) and all(elemento < 0.05 for elemento in self.error):
                    print("Localizacao alcancada")
                    if self.count == 6:
                        self.count = 0
                    else:
                        self.count += 1
                else:
                    print('contador: ', self.count,'Tentando chegar nas posicoes: X = ', self.Poseref[0],' Y = ', self.Poseref[1] , ' Z = ', self.Poseref[2], ' Yaw = ', self.Poseref[3] )
                    
                self.error_integral += self.error*self.rate.sleep_dur.to_sec()
                self.error_previous = self.error
                self.error_Ki = np.dot(Ki,controller.error_integral)
                                # Calcula as ações de controle

                controller.u_control = controller.error_Ki + Xstatesb
        
                if self.u_control[0] > 1:
                    self.error_integral[0] = 0.9*self.error_integral[0]
                    # self.u_control[0] = 1
                if self.u_control[0] < -1:
                    self.error_integral[0] = 0.9*self.error_integral[0]
                    # self.u_control[0] = -1
                if self.u_control[1] > 1:
                    self.error_integral[1] = 0.9*self.error_integral[1]
                    # self.u_control[1] = 1
                if self.u_control[1] < -1:
                    self.error_integral[1] = 0.9*self.error_integral[1]
                    # self.u_control[1] = -1
                if self.u_control[2] > 1:
                    self.error_integral[2] = 0.9*self.error_integral[2]
                    # self.u_control[2] = 1
                if self.u_control[2] < -1:
                    self.error_integral[2] = 0.9*self.error_integral[2]
                    # self.u_control[2] = -1
                if self.u_control[3] > 1:
                    self.error_integral[3] = 0.9*self.error_integral[3]
                    # self.u_control[3] = 1
                if self.u_control[3] < -1:
                    self.error_integral[3] = 0.9*self.error_integral[3]
                    # self.u_control[3] = -1
                

                vel_cmd.linear.x = (controller.u_control[0]*math.cos(controller.yaw) - controller.u_control[1]*math.sin(controller.yaw))
                vel_cmd.linear.y = (controller.u_control[0]*math.sin(controller.yaw) + controller.u_control[1]*math.cos(controller.yaw))
                vel_cmd.linear.z = controller.u_control[2]
                vel_cmd.angular.z = controller.u_control[3]
                
                pose_ref.x = self.Poseref[0]
                pose_ref.y = self.Poseref[1]
                pose_ref.z = self.Poseref[2]
                pose_ref.heading = self.Poseref[3]
                # Publica as ações de controle no tópico /cmd_vel
                self.cmd_vel_pub.publish(vel_cmd)
                self.poseref_pub.publish(pose_ref)
                # Espera o tempo restante para atingir a frequência de publicação
                self.rate.sleep()

            # vel_cmd.linear.x = 0
            # vel_cmd.linear.y = 0
            # vel_cmd.linear.z = 0
            # vel_cmd.angular.x = 0
            # vel_cmd.angular.y = 0
            # vel_cmd.angular.z = 0
            # self.cmd_vel_pub.publish(vel_cmd)
            # self.rate.sleep()
if __name__ == '__main__':
    is_turned_on = False
    try:
        controller = ControllerNode()
        controller.run()

    except rospy.ROSInterruptException:
        pass

