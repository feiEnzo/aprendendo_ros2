import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

import numpy as np
from numpy import random
import matplotlib.pyplot as plt
import time
from math import *

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info ('Definindo buffer, listener e timer para acessar as TFs.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #-------------------------------------------------------------------------------------------------

        # variables and constants
        self.raio = 0.033
        self.distancia_rodas = 0.178
        self.posee = [0, 0, 0] # x, y, theta
        self.medidas = [0, 0] # esq, dir
        self.ultimas_medidas = [0, 0] # esq, dir
        self.distancias = [0, 0]
        # mapa
        self.estado_inicial = 0
        self.mapa = [1.5, 4.5] # posição central das três “portas” existentes
        self.posee[0] = self.estado_inicial # atualiza como estado_inicial a posição x de
        #Definindo uma estimativa dos possíveis erros
        # sigma
        self.sigma_odometria = 0.2 # rad
        self.sigma_lidar = 0.175 # meters
        self.sigma_movimento = 0.002 # m
        #
        self.left_yaw = 0
        self.right_yaw = 0


    def gaussian(self, x, mean, sigma):
        return (1 / (sigma*sqrt(2*pi))) * exp(-((x-mean)**2) / (2*sigma**2))

    # update function
    def update(self):
        self.medidas[0] = self.left_yaw
        self.medidas[1] = self.right_yaw
        diff = self.medidas[0] - self.ultimas_medidas[0] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[0] = diff * self.raio + random.normal(0,0.002) # determina distância percorrida em metros e adiciona um pequeno erro
        self.ultimas_medidas[0] = self.medidas[0]
        diff = self.medidas[1] - self.ultimas_medidas[1] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[1] = diff * self.raio + random.normal(0,0.002) # determina distância percorrida em metros + pequeno erro
        self.ultimas_medidas[1] = self.medidas[1]
        # ## cálculo da dist linear e angular percorrida no timestep
        deltaS = (self.distancias[0] + self.distancias[1]) / 2.0
        deltaTheta = (self.distancias[1] - self.distancias[0]) / self.distancia_rodas
        self.posee[2] = (self.posee[2] + deltaTheta) % 6.28 # atualiza o valor Theta (diferença da divisão por 2π)
        # decomposição x e y baseado no ângulo
        deltaSx = deltaS * cos(self.posee[2])
        deltaSy = deltaS * sin(self.posee[2])
        # atualização acumulativa da posição x e y
        #self.posee[0] = self.posee[0] + deltaSx # atualiza x
        self.posee[0] = self.pose.position.x
        self.posee[0] = self.posee[0] + random.normal(0,0.002) # atualiza x
        self.posee[1] = self.posee[1] + deltaSy # atualiza y
        self.get_logger().info(f'PoseX: {self.posee[0]}')

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
       
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def get_encoder(self):
        rclpy.spin_once(self)

        try:
            self.tf_right = self.tf_buffer.lookup_transform(
                "right_center_wheel",
                "right_leg_base",
                rclpy.time.Time())

            _, _, self.right_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_right.transform.rotation.x, self.tf_right.transform.rotation.y, 
                self.tf_right.transform.rotation.z, self.tf_right.transform.rotation.w]) 

            self.get_logger().debug (
                f'yaw right_leg_base to right_center_wheel: {self.right_yaw}')

        except TransformException as ex:
            self.get_logger().error (
            f'Could not transform right_leg_base to right_center_wheel: {ex}')
        
        
        try:
            self.tf_left = self.tf_buffer.lookup_transform(
                "left_center_wheel",
                "left_leg_base",
                rclpy.time.Time())

            _, _, self.left_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_left.transform.rotation.x, self.tf_left.transform.rotation.y, 
                self.tf_left.transform.rotation.z, self.tf_left.transform.rotation.w]) 

            self.get_logger().debug (
                f'yaw left_leg_base to left_center_wheel: {self.left_yaw}')

        except TransformException as ex:
            self.get_logger().error(
            f'Could not transform left_leg_base to left_center_wheel: {ex}')


    def run(self):
         
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.5,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.ir_para_tras   = Twist(linear=Vector3(x=-0.5,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        #self.get_encoder()

        self.get_logger().info ('Ordenando o robô: "andar para frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)

        #self.get_logger().info ('Ordenando o robô: "ficar parado"')
        #self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)

        # initial graph # em roxo tudo que for relativo ao gráfico de plotagem
        x = np.linspace(-9, 9, 100) # cria um vetor x de 500 valores entre -4.5 e 4.5
        y = np.zeros(100) # cria um vetor y de 500 valores zeros
        y2 = np.zeros(100)
        y3 = np.zeros(100)
        fig, ax = plt.subplots()
        # main loop ------------------------------------------------------------------------------------------
        controle = 0
        cont = 0
        porta = 0 # robô começa em frente antes da porta 0

        self.get_logger().info ('Entrando no loop princial do nó.')
        loop = 0
        while(rclpy.ok):
            self.get_logger().info(f'Loop {loop}')
            loop += 1
            #rclpy.spin_once(self)

            self.get_logger().info ('Atualizando as distancias lidas pelo laser.')
            self.distancia_direita   = (self.laser[  0: 80]) # -90 a -10 graus
            self.distancia_frente    = (self.laser[ 80:100]) # -10 a  10 graus
            self.distancia_esquerda  = (self.laser[100:180]) #  10 a  90 graus
            self.get_logger().info(f"LASER D: {self.distancia_direita[0]} LASER E: {self.distancia_esquerda[-1]}")

            #self.get_encoder()
            #self.get_logger().info(f"YAW D: {self.right_yaw} YAW E: {self.left_yaw}")

            #PLOTAR GAUSSIANA DO ROBÔ
            #plotar em preto “b” a gaussiana da posição do robô em x (self.pose[0])
            """for i in range(len(x)):
                y[i] = self.gaussian(x[i], self.posee[0], self.sigma_movimento)
                ax.clear()
                ax.set_ylim([0, 4])
                ax.plot(x, y, color="b")"""
            leitura = (self.distancia_direita[0] + self.distancia_esquerda[-1])/2
            self.get_logger().info(f"dist: {leitura}")
            self.update() # atualiza a nova pose do robô

            self.sigma_movimento = self.sigma_movimento + 0.002 # se movimento reto, aumenta a incerteza da posição em 0.002
            if leitura > 2: # se a leitura indicar ao lado de uma porta
                media_nova = (self.mapa[porta]*self.sigma_movimento + self.posee[0]*self.sigma_lidar) / (self.sigma_movimento+self.sigma_lidar)
                sigma_novo = 1 / (1/self.sigma_movimento + 1/self.sigma_lidar)
                self.posee[0] = media_nova # a nova posição x do robô
                self.sigma_movimento = sigma_novo # novo erro gaussiano do robô
                """for i in range(len(x)): 
                    y2[i] = self.gaussian(x[i], self.mapa[porta], self.sigma_lidar)
                    ax.plot(x, y2, color="r")
                    #plt.pause(0.1) # plota em vermelho “r” a gaussiana da leitura do laser com relação à porta
                    #rclpy.spin_once(self)
                for i in range(len(x)):
                    y3[i] = self.gaussian(x[i], media_nova, sigma_novo)
                    ax.plot(x, y3, color="g")
                    #plt.pause(0.1) # plota em verde “g” a gaussiana nova após interpolação das duas gaussianas.
                    #rclpy.spin_once(self)"""
                #plt.show()
                if porta == 0: porta = 1 # altera para a próxima porta 0 → 1 ; 1 → 2
                elif porta == 1: porta = 2
                print("    ", porta)
                rclpy.spin_once(self)
                cont += 1


            if porta == 2:
                break

        self.get_logger().info ('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  

