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

# Variables and constants
raio = 0.02  # Radius of the wheel
distancia_rodas = 0.4  # Distance between the wheels
pose = [0, 0, 0]  # [x, y, theta]
medidas = [0, 0]  # [left, right]
ultimas_medidas = [0, 0]  # [left, right]
# Map with positions of the doors
estado_inicial = 0
mapa = [1.5, 4.5]  # Central position of the three "doors"
pose[0] = estado_inicial  # Initialize x position of pose

def gaussian(x, mean, sigma):
    return (1 / (sigma * sqrt(2 * pi))) * exp(-((x - mean) ** 2) / (2 * sigma ** 2))

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Defined node name as "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscriber for LaserScan
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        # Subscriber for Odometry
        self.pose_msg = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        # Publisher for robot control
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose_msg = msg.pose.pose

    def on_timer(self):
        try:
            # Right wheel transformation
            self.tf_right = self.tf_buffer.lookup_transform(
                "right_center_wheel",
                "right_leg_base",
                rclpy.time.Time()
            )

            _, _, self.right_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_right.transform.rotation.x, self.tf_right.transform.rotation.y,
                 self.tf_right.transform.rotation.z, self.tf_right.transform.rotation.w]
            )
            medidas[1] = self.right_yaw

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform right_leg_base to right_center_wheel: {ex}'
            )

        try:
            # Left wheel transformation
            self.tf_left = self.tf_buffer.lookup_transform(
                "left_center_wheel",
                "left_leg_base",
                rclpy.time.Time()
            )

            _, _, self.left_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_left.transform.rotation.x, self.tf_left.transform.rotation.y,
                 self.tf_left.transform.rotation.z, self.tf_left.transform.rotation.w]
            )
            medidas[0] = self.left_yaw

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform left_leg_base to left_center_wheel: {ex}'
            )

    def update(self):
        global pose, medidas, ultimas_medidas

        if self.pose_msg is not None:
            # Compute differences in wheel angles
            delta_theta_esq = medidas[0] - ultimas_medidas[0]
            delta_theta_dir = medidas[1] - ultimas_medidas[1]

            # Update last measurements
            ultimas_medidas[0] = medidas[0]
            ultimas_medidas[1] = medidas[1]

            # Calculate distances traveled by each wheel
            delta_s_esq = delta_theta_esq * raio
            delta_s_dir = delta_theta_dir * raio

            # Average distance traveled
            delta_s = (delta_s_dir + delta_s_esq) / 2
            delta_theta = (delta_s_dir - delta_s_esq) / distancia_rodas

            # Update robot pose
            if delta_theta != 0:
                R = delta_s / delta_theta
                pose[0] += R * (sin(pose[2] + delta_theta) - sin(pose[2]))
                pose[1] += R * (cos(pose[2]) - cos(pose[2] + delta_theta))
            else:
                pose[0] += delta_s * cos(pose[2])
                pose[1] += delta_s * sin(pose[2])

            # Update robot orientation
            pose[2] += delta_theta

            # Add Gaussian noise to position x
            pose[0] += random.normal(0, 0.002)
            self.get_logger().info(f'Updated Pose: x={pose[0]}, y={pose[1]}, theta={pose[2]}')
        else:
            self.get_logger().warning('Pose message not available for update!')

    def run(self):
        rclpy.spin_once(self)

        # Control messages
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.get_logger().info('Commanding robot to move forward.')
        self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        sigma_odometria = 0.2  # rad
        sigma_lidar = 0.175  # meters
        sigma_movimento = 0.002  # m

        self.get_logger().info('Entering main loop.')

        # Initial plot
        x = np.linspace(-10.0, 10.0, 500)
        y = np.zeros(500)
        y2 = np.zeros(500)
        y3 = np.zeros(500)
        fig, ax = plt.subplots()
        controle = 0
        cont = 0
        porta = 0  # Robot starts in front of door 0
        #################################################################################################
        parede = True
        while rclpy.ok():
            rclpy.spin_once(self)

            # Plot Gaussian of robot position
            if cont % 4 == 0:
                for i in range(len(x)):
                    y[i] = gaussian(x[i], pose[0], sigma_movimento)
                ax.clear()
                ax.set_ylim([0, 4])
                ax.plot(x, y, color="b")
                plt.pause(0.1)

            self.get_logger().debug('Updating laser distances.')

            if self.laser is None:
                continue

            self.distancia_direita = min(self.laser[0:80])  # -90 to -10 degrees
            self.distancia_frente = min(self.laser[80:100])  # -10 to 10 degrees
            self.distancia_esquerda = min(self.laser[100:180])  # 10 to 90 degrees

            if self.pose_msg is None:
                continue

            self.update()

            self.get_logger().info(f'Laser 15 {self.laser[15]}')
            self.get_logger().info(f'Laser 0 {self.laser[0]}')

            if controle == 1:
                sigma_movimento += 0.002

            self.get_logger().info(f'cont {cont}')
            self.get_logger().info(f'porta {porta}')

            if self.laser[0] > 3:
                self.pub_cmd_vel.publish(self.parar)
                media_nova = (mapa[porta] * sigma_movimento + pose[0] * sigma_lidar) / (sigma_movimento + sigma_lidar)
                self.get_logger().info(f'Correction {media_nova}')
                self.get_logger().info(f'REAL POSE {self.pose_msg.position.x}')
                sigma_novo = 1 / (1 / sigma_movimento + 1 / sigma_lidar)
                pose[0] = media_nova
                sigma_movimento = sigma_novo
                for i in range(len(x)):
                    y2[i] = gaussian(x[i], mapa[porta], sigma_lidar)
                ax.plot(x, y2, color="r")
                plt.pause(0.1)
                time.sleep(3)
                for i in range(len(x)):
                    y3[i] = gaussian(x[i], media_nova, sigma_novo)
                ax.plot(x, y3, color="g")
                plt.pause(0.1)
                time.sleep(2)
                self.pub_cmd_vel.publish(self.ir_para_frente)
                rclpy.spin_once(self)
                time.sleep(2)
                self.get_logger().info(f'porta {porta}')
                if parede:
                    parede = not parede
                    if porta == 0:
                        porta = 1
                    elif porta == 1:
                        porta = 2
                    self.get_logger().info(f'porta {porta}')

            cont += 1
            rclpy.spin_once(self)

    def __del__(self):
        self.pub_cmd_vel.publish(self.parar)
        self.get_logger().info('Finalizing node! Goodbye...')

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