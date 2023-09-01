#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from tum_ardrone.msg import filter_state
from sensor_msgs.msg import Imu
import math
import numpy as np
from std_msgs.msg import String
from tum_ardrone.srv import SetReference
from std_srvs.srv import Trigger, TriggerResponse



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

        # Cria os objetos de inscrição nos tópicos /pose e /imu
        rospy.Subscriber('/ardrone/predictedPose', filter_state, self.pose_callback)
        rospy.Subscriber('/ardrone/imu_throttle', Imu, self.imu_callback)

        # Define a frequência de publicação
        self.rate = rospy.Rate(100) # 10 Hz

        # Cria o serviço de definição de referência
        rospy.Service('setref', SetReference, self.set_reference)

        # Cria o serviço de trajetória circular
        self.command_service = rospy.Service('circ', Trigger, self.set_circ)
        
        # Cria o serviço de trajetória infinito
        self.command_service_inf = rospy.Service('inf', Trigger, self.set_inf)

        # Cria o serviço de trajetória quadrática
        self.command_service_quad = rospy.Service('quad', Trigger, self.set_quad)

        # Tempo inicial
        t = rospy.Time.now()

        # Variáveis de estado do sistema
        self.t = rospy.Time.now()
        self.center_x = 0.0
        self.center_y = 0.0
        self.radius = 0.3
        self.angular_velocity = 0.1  # Velocidade angular em rad/s
        self.pitch = 0.0
        self.dpitch = 0.0
        self.dpitch0 = 0.0
        self.x = 0.0
        self.dx = 0.0
        self.roll = 0.0
        self.droll = 0.0
        self.droll0 = 0.0
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
        self.is_service_called = False
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
        self.is_service_called = True
        self.circ_service_called = False
        self.inf_service_called = False
        self.quad_service_called = False
        response = TriggerResponse()
        response.success = True
        response.message = 'Novo Target recebido'
        self.count = 0
        return True
    # Função de serviço para definir a trajetória circular
    def set_circ(self, req):
        self.circ_service_called = True
        self.is_service_called = False
        self.inf_service_called = False
        self.quad_service_called = False
        # Retorna a resposta do serviço
        response = TriggerResponse()
        response.success = True
        response.message = 'Iniciando trajetória circular'
        self.count = 0
        return response
        # Função de serviço para definir a trajetória circular
    def set_inf(self, req):
        self.circ_service_called = False
        self.is_service_called = False
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
        self.is_service_called = False
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
        #  self.droll = -msg.angular_velocity.x*math.pi/180
        self.droll = -msg.angular_velocity.x*math.pi/180
        #self.dpitch = msg.angular_velocity.y
        #self.droll = msg.angular_velocity.x
        # self.dyaw = msg.angular_velocity.z



    # Função principal de controle
    def run(self):
        while not rospy.is_shutdown():
            # Calcula as ações de controle
            vel_cmd = Twist()
            # Parâmetros do círculo

            
            if self.is_service_called:
                self.count = 0
                # if all(elemento > -0.1 for elemento in self.error) & all(elemento < 0.1 for elemento in self.error):
                #     print("Localização alcançada")
                # else:
                #     print('Tentando chegar nas posicoes: ')
                #     print('X: ',controller.Poseref[0])
                #     print('Y: ',controller.Poseref[1])
                #     print('z: ',controller.Poseref[2])
                #     print('Yaw: ',controller.Poseref[3])
            elif self.circ_service_called:
                # Calcula a posição atual do ponto no círculo
                current_time = rospy.Time.now()
                delta_t = (current_time - self.t).to_sec()
                theta = self.angular_velocity * delta_t
                # # Calcula as coordenadas X e Y do ponto atual
                # x = self.center_x + self.radius * math.cos(theta)
                # y = self.center_y + self.radius * math.sin(theta)
                # self.Poseref[0] = x
                # self.Poseref[1] = y
                self.Poseref[0] = 0.8*math.sin(0.05*delta_t)
                self.Poseref[1] = 0.8*math.cos(0.05*delta_t)
                # self.Poseref[1] = 0
                # self.Poseref[0] = 0.7*math.sin(0.1*self.step)
                # self.Poseref[1] = 0.7*math.cos(0.1*self.step)
                self.Poseref[2] = 0.7
                self.Poseref[3] = 0 
                # if all(elemento > -0.08 for elemento in self.error) & all(elemento < 0.08 for elemento in self.error):
                #     print("Localizacao alcancada")
                #     self.step += 0.5
                # else:
                #     print('Executando trajetoria circular: X = ', self.Poseref[0],' Y = ', self.Poseref[1], ' Z = ', self.Poseref[2] )
                # if all(elemento > -0.1 for elemento in self.error[:-1]) and all(elemento < 0.1 for elemento in self.error[:-1]):
                #     print("Localizacao alcancada")
                #     self.step += 0.5
                # else:
                #     print('Executando trajetoria circular: X = ', self.Poseref[0], ' Y = ', self.Poseref[1], ' Z = ', self.Poseref[2])

            elif self.inf_service_called:
                current_time = rospy.Time.now()
                delta_t = (current_time - self.t).to_sec()
                self.Poseref[0] = math.sin(0.05*delta_t)
                self.Poseref[1] = 0.5*math.sin(0.1*delta_t)
                self.Poseref[2] = 0.7 + 0.5*math.sin(0.05*delta_t)
                self.Poseref[3] = -(math.pi/12)*math.sin(0.05*delta_t) 
                # self.Poseref[3] = 0
                # self.Poseref[0] = math.sin(0.1*self.step)
                # self.Poseref[1] = 0.5*math.sin(0.2*self.step)
                # self.Poseref[2] = 0.4 + 0.3*math.sin(0.1*self.step)
                # self.Poseref[3] = -(math.pi/12)*math.sin(0.1*self.step) 
                # if all(elemento > -0.05 for elemento in self.error) and all(elemento < 0.05 for elemento in self.error):
                #     print("Localizacao alcancada")
                #     self.step += 0.5
                # else:
                #     print('Executando trajetoria infinito: X = ', self.Poseref[0],' Y = ', self.Poseref[1] , ' Z = ', self.Poseref[2], ' Yaw = ', self.Poseref[3] )
                self.count = 0
            elif self.quad_service_called:
                # if self.count == 0:
                #     self.Poseref = [0,0,0.7,0] 
                # elif self.count == 1:
                #     self.Poseref = [1,1,0.7,0]
                # elif self.count == 2:
                #     self.Poseref = [-1,1,0.7,0]
                # elif self.count == 3:
                #     self.Poseref = [-1,-1,0.7,0]
                # elif self.count == 4:
                #     self.Poseref = [1,-1,0.7,0]
                # elif self.count == 5:
                #     self.Poseref = [1,1,0.7,0]  
                # elif self.count == 6:
                #     self.Poseref = [0,0,0.7,0] 
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

            
  

            #//y yponto x xponto -theta -thetaponto -rolll -rollponto z zponto -yaw -yawponto ex ey ez -eyaw

            controller.Xstates = [controller.pitch, controller.dpitch, controller.x, controller.dx, controller.roll, controller.droll, controller.y, controller.dy, controller.z, controller.dz, controller.yaw, controller.dyaw]
            
        
            controller.Posereal = [controller.x, controller.y, controller.z, controller.yaw]
            

            # K = [[-3.06, -0.46, -3.12, -1.59, 0, 0, 0, 0, 0, 0, 0, 0],
            #      [0, 0, 0, 0, -3.42, -0.48, 3.63, 1.64, 0, 0, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, -5.15, -1.59, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.19, -0.08]] #K original Rodrigo
            K = [[-1.9147, -0.6052, -0.8325, -0.9071, 0, 0, 0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, -1.9251, -0.6058, 0.6746, 0.9130, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 0, -2.0768, -0.6069, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5572, -0.0835]] #K LQR lento
            # K = [[-4.7413, -1.0060, -2.3713, -1.8829, 0, 0, 0, 0, 0, 0, 0, 0],
            #      [0, 0, 0, 0, -4.7550, -1.0059, 2.0796, 1.9015, 0, 0, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, -2.7774, -0.8259, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.8125, -0.1247]] #K LQR rápido
            # K = [[-9.6070, -1.5969, -7.8896, -3.2814, 0, 0, 0, 0, 0, 0, 0, 0],
            #      [0, 0, 0, 0, -14.1210, -2.6604, 9.2171, 3.9963, 0, 0, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, -3.6448, -2.2677, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.8984, -0.0759]] #K ajuste eixos

            # K = [[-2.16170249236764, -0.462650799210097, -1.30821736744336, -0.748741296619625, 0, 0, 0, 0, 0, 0, 0, 0],
            #         [0, 0, 0, 0, -2.55114763776489, -0.852101536979317, 1.286571397078, 0.778584973225811, 0, 0, 0, 0],
            #         [0, 0, 0, 0, 0, 0, 0, 0, -1.00256246680517, -0.373539749563795, 0, 0],
            #        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.597680332323163, -0.0426529302084393]] #LQR pós-ajuste param

            #K = [[-28.40490000000 , -3.62390000000000 ,	-29.0442000000000 ,	-12.2318000000000, 0, 0, 0, 0, 0, 0, 0, 0],
            #      [0, 0, 0, 0, -29.2154000000000 ,	-3.64860000000000 ,	29.0348000000000 ,	12.8469000000000, 0, 0, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, -4.80270000000000,	-1.58600000000000, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9.13400000000000 ,	-3.09680000000000]] #K humberto

 
            # K = [[-2.3300, -0.4578, -1.7240, -0.8035, 0, 0, 0, 0, 0, 0, 0, 0],
            #      [0, 0, 0, 0, -2.3833, -0.6595, 1.4823, 0.7344, 0, 0, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, -2.0816, -0.6692, 0, 0],
            #      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.6427, -0.4653]] #LQR simples
            K = [[-2.4496, -0.5186, -1.5088, -0.8481, 0, 0, 0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, -2.9098, -0.9564, 1.4914, 0.8853, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 0, -1.0574, -0.3941, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.9184, -0.1469]] #LQR robusto


            # Ki= [[2.86, 0, 0, 0],
            #      [0,-3.43, 0, 0],
            #      [0, 0, 4.47,0],
                #  [0, 0, 0, 1.0]] #Ki original Rodrigo
            # Ki= [[0.2274, 0, 0, 0],
            #      [0,-0.1666, 0, 0],
            #      [0, 0, 0.9764,0],
            #      [0, 0, 0, 0.2774]] #LQR lento 
            # Ki= [[4.3495, 0, 0, 0],
            #      [0,-4.7616, 0, 0],
            #      [0, 0, 1.0438,0],
            #      [0, 0, 0, 0.5123]] #LQR ajuste eixos                
            # Ki= [[0.9512, 0, 0, 0],
            #      [0,-0.7911, 0, 0],
            #      [0, 0, 1.4398,0],
            #      [0, 0, 0, 0.5168]] #LQR rápido
            # // vector_k11 = TooN::makeVector(0,0,-2.7755, -1.3071,3.8102, 0.7529,0,0,0,0,0,0,0,1.1739,0,0); // Gabriel agora vai
            # // vector_k21 = TooN::makeVector(2.1706,1.1396,0,0,0,0,3.8906,1.1801,0,0,0,0,-0.7387,0,0,0); // Gabriel agora vai
            #// vector_k31 = TooN::makeVector(0,0,0,0,0,0,0,0,-4.5445, -1.2593,0,0,0,0,3.7957,0); // Gabriel agora vai
            #	// vector_k41 = TooN::makeVector(0,0,0,0,0,0,0,0,0,0,0.3186, 0.1085,0,0,0,-0.0644); // Gabriel agora vai
            # Ki = [[0.190173350319216, 0, 0, 0],
            #        [0, -0.191442697220180, 0, 0],
            #        [0, 0, 0.187761724291528, 0],
            #        [0, 0, 0, 0.541470446517724]] #LQR pós-ajuste param
            #Ki= [[31.6228, 0, 0, 0],
            #      [0,-31.6228, 0, 0],
            #      [0, 0, 4.4721,0],
            #      [0, 0, 0, 10]] #LQR humberto
            # Ki= [[0.7476, 0, 0, 0],
            #      [0,-0.6028, 0, 0],
            #      [0, 0, 1.0626,0],
            #      [0, 0, 0, 0.1387]] #Ki simples
            Ki= [[0.2587, 0, 0, 0],
                 [0,-0.2608, 0, 0],
                 [0, 0, 0.1973,0],
                 [0, 0, 0, 0.4797]] #Ki robusto


            Xstatesb = np.dot(K,controller.Xstates)
            # print(Xstatesb)
            self.error = np.subtract(controller.Poseref, controller.Posereal)

            if all(elemento > -0.05 for elemento in self.error) and all(elemento < 0.05 for elemento in self.error):
                print("Localizacao alcancada")
                if self.count == 6:
                    self.count = 0
                else:
                    self.count += 1
            else:
                print('contador: ', self.count,'Tentando chegar nas posicoes: X = ', self.Poseref[0],' Y = ', self.Poseref[1] , ' Z = ', self.Poseref[2], ' Yaw = ', self.Poseref[3] )
                
            # print(self.Poseref)
            # print(self.Posereal)
            # print(self.error)
            # Integral do erro utilizando a regra do trapézio
            # self.error_integral += ((self.error + self.error_previous) / 2 )*self.rate.sleep_dur.to_sec()
            self.error_integral += self.error*self.rate.sleep_dur.to_sec()
            self.error_previous = self.error
            self.error_Ki = np.dot(Ki,controller.error_integral)
            
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
            # vel_cmd.linear.x = 0 
            # vel_cmd.linear.y = 0
            #vel_cmd.linear.x = (controller.u_control[0]*math.cos(controller.yaw) + controller.u_control[1]*math.sin(controller.yaw))
            #vel_cmd.linear.y = (-controller.u_control[0]*math.sin(controller.yaw) + controller.u_control[1]*math.cos(controller.yaw))

            vel_cmd.linear.z = controller.u_control[2]
            vel_cmd.angular.z = controller.u_control[3]




        
            # Publica as ações de controle no tópico /cmd_vel
            self.cmd_vel_pub.publish(vel_cmd)

            # Espera o tempo restante para atingir a frequência de publicação
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ControllerNode()
        controller.run()

    except rospy.ROSInterruptException:
        pass

