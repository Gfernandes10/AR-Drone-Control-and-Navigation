#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import numpy as np
from std_msgs.msg import String
from controllers.srv import SetReference
from controllers.srv import CircParameters
from controllers.msg import poseref
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pdb
import time

def angle_from_to_2(angle, min_angle, max_angle):
    while angle < min_angle:
        angle += 360
    while angle >= max_angle:
        angle -= 360
    return angle


class ControllerNode:

    def __init__(self):
        # pdb.set_trace()
        # Inicializa o node do ROS
        rospy.init_node('controller_node', anonymous=True)
        # Cria o objeto de publicação no tópico /cmd_vel
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        # self.poseref_pub = rospy.Publisher('/poseref', poseref, queue_size=10)
        self.poseref_pub = rospy.Publisher('/poseref', poseref, queue_size=10)
        self.erros_pub = rospy.Publisher('/erros', poseref, queue_size=10)
        # Cria os objetos de inscrição nos tópicos /pose e /imu
        # rospy.Subscriber('/ardrone/predictedPose', filter_state, self.pose_callback)
        
        # rospy.Subscriber('/linear_system/pose', Odometry, self.pose_callback)
        # rospy.Subscriber('/bebop2/odometry_sensor1/odometry', Odometry, self.pose_callback)
        
        rospy.Subscriber('/filtered_pose', Odometry, self.pose_callback)
        # rospy.Subscriber('/ardrone/imu_throttle', Imu, self.imu_callback)
        # rospy.Subscriber('/ardrone/imu', Imu, self.imu_callback)
        # Define a frequência de publicação
        rospy.Subscriber('/activate_lqr_controller', Empty, self.empty_callback)
        # self.rate = rospy.Rate(100) # 100 Hz,
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
        self.Init =True
        
        self.t = rospy.Time.now()
        self.timechange=0.000
        self.last_time=0.0000
        self.circ_radius = 0.8  #Radius of the circle
        self.circ_angular_velocity = 0.05  # angular velocity of the circle in rad/s
        self.circ_z = 0.7  #Height of the circle in meters
        self.pitch = 0.0
        self.dpitch = 0.0
        self.previous_time = rospy.Time.now()
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
        # self.Poseref = [0.0, 0.0, 0.0, 0.0]
        self.Xstates = [0.0, 0.0, 0.0, 0.0]
        self.Ystates = [0.0, 0.0, 0.0, 0.0]
        self.Zstates = [0.0, 0.0]
        self.Yawstates = [ 0.0, 0.0]
        self.XstatesFb = 0.0
        self.YstatesFb = 0.0
        self.ZstatesFb = 0.0
        self.YawstatesFb = 0.0
        # self.Posereal = [0.0,0.0,0.0,0.0]
        # self.Poseref = [0.0,0.0,0.0,0.0]
        self.error = [0.0,0.0,0.0,0.0]
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
        # self.Kx   = [-0.0087,-0.0062,-0.7636,-0.1476]
        self.Kx   = [-0.0287,-0.0262,-0.7636,-0.1476]
        self.Kix  = 0.1846
        self.Kix  = 0.00246
        # self.Ky   = [0.0199,0.0130,-0.5906,-0.1791]
        self.Ky   = [0.1199,0.1130,-0.5906,-0.1791]
        self.Kiy  = 0.1778
        self.Kiy  = 0.00278
        # self.Kz   = [-0.6558,-0.7091]
        # self.Kiz  = 0.1574
        self.Kz   = [-3.6558,-3.7091]
        self.Kiz  = 0.01574
        self.Kyaw = [-3.0674,-3.3309]
        # self.Kyaw = [-7.0674,-5.3309]
        self.Kiyaw= 0.0388
        self.vel_cmd = Twist()
        self.pose_ref = poseref()
        self.erros_ref = poseref()
        self.vel_cmd.linear.x = 0
        self.vel_cmd.linear.y = 0
        self.vel_cmd.linear.z = 0
        self.vel_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.vel_cmd)
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
    def pose_callback(self, Kfpos):
 

        if self.onoff:

            
            # x dx theta dtheta y dy roll droll z dz yaw dyaw
            quaternion = [Kfpos.pose.pose.orientation.x, Kfpos.pose.pose.orientation.y, Kfpos.pose.pose.orientation.z, Kfpos.pose.pose.orientation.w]
            euler_angles = euler_from_quaternion(quaternion) 
            self.x      = Kfpos.pose.pose.position.x
            self.dx     = Kfpos.twist.twist.linear.x
            self.pitch  = euler_angles[1]
            self.dpitch = Kfpos.twist.twist.angular.y
            self.y  = Kfpos.pose.pose.position.y
            self.dy = Kfpos.twist.twist.linear.y
            self.roll  = euler_angles[0]
            self.droll = Kfpos.twist.twist.angular.x
            self.z = Kfpos.pose.pose.position.z
            self.dz = Kfpos.twist.twist.linear.z
            dcm10 = 2 * (Kfpos.pose.pose.orientation.x * Kfpos.pose.pose.orientation.y + Kfpos.pose.pose.orientation.w*Kfpos.pose.pose.orientation.z)
            dcm00 = (Kfpos.pose.pose.orientation.w*Kfpos.pose.pose.orientation.w) + (Kfpos.pose.pose.orientation.x*Kfpos.pose.pose.orientation.x) - (Kfpos.pose.pose.orientation.y*Kfpos.pose.pose.orientation.y)  - (Kfpos.pose.pose.orientation.z*Kfpos.pose.pose.orientation.z)
            yaw = math.atan2(dcm10,dcm00)
            self.yaw = angle_from_to_2(yaw*180/math.pi, -180, 180)*math.pi/180
            self.dyaw = Kfpos.twist.twist.angular.z
            
            self.Posereal = [self.x, self.y, self.z, self.yaw]

            self.Xstates = [self.pitch, self.dpitch, self.x, self.dx]
            self.Ystates = [self.roll, self.droll, self.y, self.dy]
            self.Zstates = [self.z, self.dz]
            self.Yawstates = [self.yaw, self.dyaw]

            self.XstatesFb = np.dot(self.Kx,self.Xstates)
            self.YstatesFb = np.dot(self.Ky,self.Ystates)
            self.ZstatesFb = np.dot(self.Kz,self.Zstates)
            self.YawstatesFb = np.dot(self.Kyaw,self.Yawstates)

            if self.Init:
                self.vel_cmd.linear.x = 0
                self.vel_cmd.linear.y = 0
                self.vel_cmd.linear.z = 0
                self.vel_cmd.angular.z = 0
                self.cmd_vel_pub.publish(self.vel_cmd)
                self.error_integral = np.zeros(4)
                self.Poseref = [self.x,self.y,self.z,self.yaw]
                self.Init = False 

            self.error = np.subtract(self.Poseref, self.Posereal)

            if all(elemento > -0.1 for elemento in self.error) and all(elemento < 0.1 for elemento in self.error):
                print("Localizacao alcancada")
                self.error = [0.0, 0.0, 0.0, 0.0]
                if self.count == 6:
                    self.count = 0
                else:
                    self.count += 1
            else:
                print('contador: ', self.count,'Tentando chegar nas posicoes: X = ', self.Poseref[0],' Y = ', self.Poseref[1] , ' Z = ', self.Poseref[2], ' Yaw = ', self.Poseref[3]*180/math.pi )
                print("Erros atuais: X = ", self.error[0], ' Y = ', self.error[1], 'Z = ', self.error[2], 'Yaw = ', self.error[3])
   
            self.erros_ref.x = self.error[0]
            self.erros_ref.y = self.error[1]
            self.erros_ref.z = self.error[2]
            self.erros_ref.heading = self.error[3]
            self.erros_pub.publish(self.erros_ref)
            

        #//y yponto x xponto -theta -thetaponto -rolll -rollponto z zponto -yaw -yawponto ex ey ez -eyaw
    # Função de callback para o tópico /imu
    def empty_callback(self,msg):
        global is_turned_on
        is_turned_on = not is_turned_on
        if is_turned_on:
            rospy.loginfo("Controlador ligado")
            self.onoff = True
        else:
            rospy.loginfo("Controlador desligado")
            self.onoff = False

    def target_func(self):
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
            self.count = 180
        elif self.quad_service_called:
            if self.count == 0:
                self.Poseref = [0,0,0.4,180]
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
            # self.Poseref = [self.x,self.y,self.z,self.yaw]
            self.count = 0
        


    # Função principal de controle
    def run(self):
        # pdb.set_trace()
        self.vel_cmd.linear.x = 0
        self.vel_cmd.linear.y = 0
        self.vel_cmd.linear.z = 0
        self.vel_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.vel_cmd)
        self.error_integral = np.zeros(4)
        self.rate = rospy.Rate(200) 

        while not rospy.is_shutdown(): 
            
            self.now = int(round(time.time() * 1000))
            self.timechange=self.now-self.last_time

            self.target_func() 
        
            self.error_integral[0] += self.error[0]
            self.error_integral[1] += self.error[1]
            self.error_integral[2] += self.error[2]
            self.error_integral[3] += self.error[3]
            # self.error_integral = self.error

            self.error_Ki[0] = np.dot(self.Kix,self.error_integral[0])
            self.error_Ki[1] = np.dot(self.Kiy,self.error_integral[1])
            self.error_Ki[2] = np.dot(self.Kiz,self.error_integral[2])
            self.error_Ki[3] = np.dot(self.Kiyaw,self.error_integral[3])

            self.u_control[0] = self.error_Ki[0] + self.XstatesFb 
            self.u_control[1] = self.error_Ki[1] + self.YstatesFb 
            self.u_control[2] = self.error_Ki[2] + self.ZstatesFb 
            self.u_control[3] = self.error_Ki[3] + self.YawstatesFb

            if self.u_control[0] > 1:
                self.error_integral[0] = 0.9*self.error_integral[0]
                self.error_Ki[0] = np.dot(self.Kix,self.error_integral[0])
                self.u_control[0] = self.error_Ki[0] + self.XstatesFb 
                self.u_control[0] = 1
            if self.u_control[0] < -1:
                self.error_integral[0] = 0.9*self.error_integral[0]
                self.error_Ki[0] = np.dot(self.Kix,self.error_integral[0])
                self.u_control[0] = self.error_Ki[0] + self.XstatesFb                 
                self.u_control[0] = -1
            if self.u_control[1] > 1:
                self.error_integral[1] = 0.9*self.error_integral[1]
                self.error_Ki[1] = np.dot(self.Kiy,self.error_integral[1])
                self.u_control[1] = self.error_Ki[1] + self.YstatesFb 
                self.u_control[1] = 1
            if self.u_control[1] < -1:
                self.error_integral[1] = 0.9*self.error_integral[1]
                self.error_Ki[1] = np.dot(self.Kiy,self.error_integral[1])
                self.u_control[1] = self.error_Ki[1] + self.YstatesFb 
                # self.u_control[1] = -1
            if self.u_control[2] > 1:
                self.error_integral[2] = 0.9*self.error_integral[2]
                self.error_Ki[2] = np.dot(self.Kiz,self.error_integral[2])
                self.u_control[2] = self.error_Ki[2] + self.ZstatesFb 
                # self.u_control[2] = 1
            if self.u_control[2] < -1:
                self.error_integral[2] = 0.9*self.error_integral[2]
                self.error_Ki[2] = np.dot(self.Kiz,self.error_integral[2])
                self.u_control[2] = self.error_Ki[2] + self.ZstatesFb 
                # self.u_control[2] = -1
            if self.u_control[3] > 1:
                self.error_integral[3] = 0.9*self.error_integral[3]
                self.error_Ki[3] = np.dot(self.Kiyaw,self.error_integral[3])
                self.u_control[3] = self.error_Ki[3] + self.YawstatesFb
                # self.u_control[3] = 1
            if self.u_control[3] < -1:
                self.error_integral[3] = 0.9*self.error_integral[3]
                self.error_Ki[3] = np.dot(self.Kiyaw,self.error_integral[3])
                self.u_control[3] = self.error_Ki[3] + self.YawstatesFb
                self.u_control[3] = -1



            # self.vel_cmd.linear.x = (self.u_control[0]*math.cos(self.yaw) - self.u_control[1]*math.sin(self.yaw))
            # self.vel_cmd.linear.y = (self.u_control[0]*math.sin(self.yaw) + self.u_control[1]*math.cos(self.yaw))
            # self.vel_cmd.linear.z = self.u_control[2]        
            # self.vel_cmd.angular.z = self.u_control[3]

            self.vel_cmd.linear.x = self.u_control[0]
            self.vel_cmd.linear.y = self.u_control[1]
            self.vel_cmd.linear.z = self.u_control[2]        
            self.vel_cmd.angular.z = self.u_control[3]

            # self.vel_cmd.linear.x = 0
            # self.vel_cmd.linear.y = 0
            # self.vel_cmd.linear.z = 0
            # self.vel_cmd.angular.z = 0

            # self.pose_ref.x = self.Poseref[0]
            self.pose_ref.x = self.Poseref[0]
            # self.pose_ref.y = self.Poseref[1]
            self.pose_ref.y = self.Poseref[1]
            self.pose_ref.z = self.Poseref[2]
            self.pose_ref.heading = self.Poseref[3]

            # Publica as ações de controle no tópico /cmd_vel
            self.cmd_vel_pub.publish(self.vel_cmd)
            self.poseref_pub.publish(self.pose_ref)                                          

            self.rate.sleep()
            self.last_time=self.now

                
if __name__ == '__main__':
    is_turned_on = False
    try:
        controller = ControllerNode()
        controller.Poseref = [0.0, 0.0, 0.0, 0.0]  
        controller.vel_cmd.linear.x = 0
        controller.vel_cmd.linear.y = 0
        controller.vel_cmd.linear.z = 0
        controller.vel_cmd.angular.z = 0
        controller.cmd_vel_pub.publish(controller.vel_cmd)
        controller.error_integral = np.zeros(4)
        controller.run()

    except rospy.ROSInterruptException:
        controller.vel_cmd.linear.x = 0
        controller.vel_cmd.linear.y = 0
        controller.vel_cmd.linear.z = 0
        controller.vel_cmd.angular.z = 0
        controller.cmd_vel_pub.publish(controller.vel_cmd)
        pass
