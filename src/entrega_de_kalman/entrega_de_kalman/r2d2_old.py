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

from math import *
import numpy as np
import matplotlib.pyplot as plt
from numpy import random
import time


# variables and constants
raio = 0.02
distancia_rodas = 0.4
pose = [0, 0, 0] # x, y, theta
medidas = [0, 0] # esq, dir
ultimas_medidas = [0, 0] # esq, dir
distancias = [0, 0]
# mapa
estado_inicial = 0
mapa = [1.5, 4.5, 7.5] # posição central das três “portas” existentes
pose[0] = estado_inicial # atualiza como estado_inicial a posição x de pose


def gaussian(x, mean, sigma):
 return (1 / (sigma*sqrt(2*pi))) * exp(-((x-mean)**2) / (2*sigma**2))

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        #self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        #self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        #self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        #self.get_logger().info ('Definindo buffer, listener e timer para acessar as TFs.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
       
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def on_timer(self):
        try:
            self.tf_right = self.tf_buffer.lookup_transform(
                "right_center_wheel",
                "right_leg_base",
                rclpy.time.Time())

            _, _, self.right_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_right.transform.rotation.x, self.tf_right.transform.rotation.y, 
                self.tf_right.transform.rotation.z, self.tf_right.transform.rotation.w]) 
            if(self.right_yaw<0): 
                self.right_yaw = -1*self.right_yaw + ultimas_medidas[1]
            medidas[1] = self.right_yaw
            #self.get_logger().info (
            #    f'yaw right_leg_base to right_center_wheel: {self.right_yaw}')

        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform right_leg_base to right_center_wheel: {ex}')
        
        
        try:
            self.tf_left = self.tf_buffer.lookup_transform(
                "left_center_wheel",
                "left_leg_base",
                rclpy.time.Time())

            _, _, self.left_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_left.transform.rotation.x, self.tf_left.transform.rotation.y, 
                self.tf_left.transform.rotation.z, self.tf_left.transform.rotation.w]) 
            if(self.left_yaw<0): 
                self.left_yaw = -1*self.left_yaw + ultimas_medidas[0]
            medidas[0] = self.left_yaw
            #self.get_logger().info (
            #    f'yaw left_leg_base to left_center_wheel: {self.left_yaw}')

        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform left_leg_base to left_center_wheel: {ex}')

    # update function
    def update(self):
        # Atualização acumulativa da posição x e y
        if self.pose is not None:
            pose[0] = self.pose.position.x  # Atualiza x com a posição atual
            pose[0] += random.normal(0, 0.002)  # Adiciona um ruído gaussiano
            self.get_logger().info(f'Predição {pose[0]}')
        else:
            self.get_logger().warning('Pose não disponível para atualização!')

    def run(self):

        rclpy.spin_once(self)

        # Mensagens de controle do robô
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.get_logger().info('Ordenando o robô: "ir para a frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        # sigma
        sigma_odometria = 0.2  # rad
        sigma_lidar = 0.175  # meters
        sigma_movimento = 0.002  # m

        self.get_logger().info('Entrando no loop principal do nó.')

        # Gráfico inicial
        x = np.linspace(-10.0, 10.0, 500)  # Cria um vetor x de 500 valores entre -4.5 e 4.5
        y = np.zeros(500)  # Cria um vetor y de 500 valores zeros
        y2 = np.zeros(500)
        y3 = np.zeros(500)
        fig, ax = plt.subplots()
        controle = 0
        cont = 0
        porta = 0  # Robô começa em frente antes da porta 0

        while rclpy.ok():
            rclpy.spin_once(self)

            # Plotar gaussiana do robô
            if cont % 4 == 0:  # A cada 4 passos, plotar em preto “b” a gaussiana da posição do robô em x (pose[0])
                for i in range(len(x)):
                    y[i] = gaussian(x[i], pose[0], sigma_movimento)
                ax.clear()
                ax.set_ylim([0, 4])
                ax.plot(x, y, color="b")
                plt.pause(0.1)

            self.get_logger().debug('Atualizando as distâncias lidas pelo laser.')

            if self.laser is None:
                continue
            self.distancia_direita = min(self.laser[0:80])  # -90 a -10 graus
            self.distancia_frente = min(self.laser[80:100])  # -10 a 10 graus
            self.distancia_esquerda = min(self.laser[100:180])  # 10 a 90 graus

            if self.pose is None:
                continue

            self.update()

            self.get_logger().info(f'Laser 15 {self.laser[15]}')
            self.get_logger().info(f'Laser 0 {self.laser[0]}')
            if controle == 1:
                sigma_movimento += 0.002  # Se movimento reto, aumenta a incerteza da posição em 0.002
            self.get_logger().info(f'cont {cont}')
            self.get_logger().info(f'porta {porta}')
            if self.laser[0] > 3:
                self.pub_cmd_vel.publish(self.parar)
                media_nova = (mapa[porta] * sigma_movimento + pose[0] * sigma_lidar) / (sigma_movimento + sigma_lidar)
                self.get_logger().info(f'Correção {media_nova}')
                self.get_logger().info(f'POSE REAL {self.pose.position.x}')
                sigma_novo = 1 / (1 / sigma_movimento + 1 / sigma_lidar)
                pose[0] = media_nova  # Nova posição x do robô
                sigma_movimento = sigma_novo  # Novo erro gaussiano do robô
                for i in range(len(x)):
                    y2[i] = gaussian(x[i], mapa[porta], sigma_lidar)
                ax.plot(x, y2, color="r")
                plt.pause(0.1)  # Plota em vermelho “r” a gaussiana da leitura do laser com relação à porta
                time.sleep(3)
                for i in range(len(x)):
                    y3[i] = gaussian(x[i], media_nova, sigma_novo)
                ax.plot(x, y3, color="g")
                plt.pause(0.1)  # Plota em verde “g” a gaussiana nova após interpolação das duas gaussianas.
                time.sleep(2)
                self.pub_cmd_vel.publish(self.ir_para_frente)
                rclpy.spin_once(self)
                time.sleep(2)
                self.get_logger().info(f'porta {porta}')
                if porta == 0:
                    porta = 1  # Altera para a próxima porta 0 → 1 ; 1 → 2
                elif porta == 1:
                    porta = 2
                self.get_logger().info(f'porta {porta}')

            cont += 1
            rclpy.spin_once(self)


    


    # Destrutor do nó
    def __del__(self):
        self.pub_cmd_vel.publish(self.parar)
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
