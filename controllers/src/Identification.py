#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from filterpy.kalman import KalmanFilter
import numpy as np
from math import radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from controllers.msg import poseref
import sympy as sp

def angle_from_to_2(angle, min_angle, max_angle):
    while angle < min_angle:
        angle += 360
    while angle >= max_angle:
        angle -= 360
    return angle

class LinearSystemSimulator:
    def __init__(self):
        rospy.init_node('Identification')
        self.callback_called = False
        #Parâmetros
        omega_theta = 4.0
        zeta_theta  = 0.2
        g           = 9.81
        Cx          = 5.0
        K_theta     = 1.0
        theta_max   = 0.29
        omega_phi   = 4.5
        zeta_phi    = 0.15
        Cy          = 3.0
        K_phi       = 1.0
        phi_max     = -0.29
        tal_z       = 1.6
        K_z         = 2.2      
        z_max       = 1.8     
        tal_psi     = 0.2
        K_psi       = 0.4         
        psi_max     = 1.74   
        
    
        # Extraindo as matrizes A, B, C, D
        # A_matrix, B_matrix, C_matrix, D_matrix = sp.linear_systems.systems.sys_to_ss(sys_ss, s)

        self.Ax = np.array([[0.0, 1.0, 0.0, 0.0], 
                            [-omega_theta**2, -2*zeta_theta*omega_theta, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0],
                            [0, 0.0, 0.0, -Cx]])
        self.Ay = np.array([[0.0, 1.0, 0.0, 0.0], 
                            [-omega_phi**2, -2*zeta_phi*omega_phi, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0],
                            [0, 0.0, 0.0, -Cy]]) 
        self.Az = np.array([[0.0, 1.0], 
                            [0.0, -1/tal_z]]) 
        self.Ayaw = np.array([[0.0, 1.0], 
                            [0.0, -1/tal_psi]]) 
        print(self.Ax)
        print(self.Ay)
        print(self.Az)
        print(self.Ayaw)
        # Parâmetros do sistema
        # Compondo as matrizes A e B
        
        # B = np.block([[self.Bx], [np.zeros((3,4))],
                    #   [self.Bx], [np.zeros((3,4))]])
        self.Bx = np.array([[0.0], [K_theta*(omega_theta**2)*theta_max], [0], [g]]) 
        self.By = np.array([[0.0], [K_phi*(omega_phi**2)*phi_max], [0], [g]])
        self.Bz = np.array([[0.0],[(K_z*z_max)/tal_z]]) 
        self.Byaw = np.array([[0.0],[(K_psi*psi_max)/tal_psi]]) 

        # Estado inicial
        self.x = 0.0
        self.dx = 0.0
        self.pitch = 0.0
        self.dpitch = 0.0
        self.y = 0.0
        self.dy = 0.0
        self.roll = 0.0
        self.droll = 0.0
        self.z = 0.0
        self.dz = 0.0
        self.yaw = 0.0
        self.dyaw = 0.0
        self.u = [0.0, 0.0, 0.0,0.0]
        self.x_estados = np.array([self.pitch,self.dpitch,self.x,self.dx,self.roll,self.droll,self.y,self.dy,self.z,self.dz,self.yaw,self.dyaw])
        self.Xstates = np.array([[self.pitch], [self.dpitch], [self.x], [self.dx]])
        print(self.Xstates)
        self.Ystates = np.array([[self.roll], [self.droll], [self.y], [self.dy]])
        self.Zstates = np.array([[self.z], [self.dz]])
        self.YawStates = np.array([[self.yaw], [self.dyaw]])

        # Publicadores
        self.pose_publisher = rospy.Publisher('/linear_system/pose', Odometry, queue_size=10)
        self.poseref_pub = rospy.Publisher('/poseref', poseref, queue_size=10)
        # self.cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/bebop/cmd_vel', Twist, self.cmdvel_callback)
        # Taxa de publicação
        self.rate = rospy.Rate(200)  # 10 Hz

        # rospy.Subscriber('/filtered_pose', Odometry, self.odometry_callback)

    def simulate_step(self, u,dt):
        # Modelo de espaço de estado discreto
        # self.x_estados = np.dot(self.A, self.x_estados) + np.dot(self.B, u)
        self.Xstates = self.Xstates +dt*(np.dot(self.Ax, self.Xstates) + np.dot(self.Bx, u[0]))
        self.Ystates = self.Ystates +dt*(np.dot(self.Ay, self.Ystates) + np.dot(self.By, u[1]))
        self.Zstates = self.Zstates +dt*(np.dot(self.Az, self.Zstates) + np.dot(self.Bz, u[2]))
        self.YawStates = self.YawStates +dt*(np.dot(self.Ayaw, self.YawStates) + np.dot(self.Byaw, u[3]))
        # self.x_estados = np.array([self.Xstates[0],self.Xstates[1],self.Xstates[2],self.Xstates[3],
        #                             self.Ystates[0],self.Ystates[1],self.Ystates[2],self.Ystates[3], 
        #                             self.Zstates[0],self.Zstates[1],
        #                             self.YawStates[0],self.YawStates[1]])
        # self.x_estados = self.x_estados + dt * (np.dot(self.A, self.x_estados) + np.dot(self.B, u))
    def cmdvel_callback(self,cmd_vel):
        self.u[0] = cmd_vel.linear.x
        self.u[1] = cmd_vel.linear.y
        self.u[2] = cmd_vel.linear.z
        self.u[3] = cmd_vel.angular.z
    def publish_pose(self):       
        # Publicar a pose filtrada            
        pose_msg = Odometry()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.pose.position.x = self.Xstates[2]
        pose_msg.pose.pose.position.y = self.Ystates[2]
        pose_msg.pose.pose.position.z = self.Zstates[1]
        euler_angles = [self.Ystates[0], self.Xstates[0], self.YawStates[0]]
        quaternion = quaternion_from_euler(euler_angles[0],euler_angles[1],np.radians(angle_from_to_2(np.degrees(euler_angles[2]),-180,180)))
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        pose_msg.twist.twist.linear.x = self.Xstates[3]
        pose_msg.twist.twist.linear.y = self.Ystates[3]
        pose_msg.twist.twist.linear.z = self.Zstates[1]
        pose_msg.twist.twist.angular.x = self.Ystates[1]
        pose_msg.twist.twist.angular.y = self.Xstates[1]
        pose_msg.twist.twist.angular.z = self.YawStates[1]
        self.pose_publisher.publish(pose_msg)
    def generate_input_signal(self, t):
        # Sinal de entrada composto por duas senoidais
        # print(t)        
        u1 = 0.5 * np.sin(2 * np.pi * t / 7.5)
        u2 = 0.3 * np.sin(2 * np.pi * t / (0.2*7.5))
        u = u1 + u2
        if t>20:
            u = 0
            print("Fim")
        return np.array([0.0, 0.0, 0.0, u])  # Altere conforme necessário

    def run(self):
        t0 = 0
        pose_ref = poseref()
        cmd_vel = Twist()
        while not rospy.is_shutdown():
            # Simula um passo do sistema
            t = t0 + 0.005
            t0 = t
            # input_signal = self.generate_input_signal(t)
            # pose_ref.x = input_signal[0]
            # pose_ref.y = input_signal[1]
            # pose_ref.z = input_signal[2]
            # pose_ref.heading = input_signal[3]
            # self.poseref_pub.publish(pose_ref)
            # cmd_vel.linear.x = input_signal[0]
            # cmd_vel.linear.y = input_signal[1]
            # cmd_vel.linear.z = input_signal[2]
            # cmd_vel.angular.z= input_signal[3]
            # self.cmd_pub.publish(cmd_vel)
            input_signal = self.u
            # input_signal = np.array([0.0, 1.0, 0.0, 0.0])  # Pode ser substituído por qualquer sinal de entrada desejado
            self.simulate_step(input_signal,0.005)

            # Publica o estado atual como pose
            self.publish_pose()
            
            # Espera até a próxima iteração
            self.rate.sleep()                  
if __name__ == '__main__':
    try:
        system_simulator  = LinearSystemSimulator()
        system_simulator.run()
    except rospy.ROSInterruptException:
        pass
