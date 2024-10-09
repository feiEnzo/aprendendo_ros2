import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import tf_transformations

import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi



class MAPEADOR(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Defined node name as "MAPEADOR"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscriber for LaserScan
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        # Subscriber for Odometry
        self.pose_msg = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.x = None # metros
        self.y = None # metros
        self.a = None # radiandos


    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
    
    def listener_callback_odom(self, msg):
        self.pose_msg = msg.pose.pose
        self.x = self.pose_msg.position.x
        self.y = self.pose_msg.position.y

        _, _, self.a = tf_transformations.euler_from_quaternion([self.pose_msg.orientation.x, self.pose_msg.orientation.y, self.pose_msg.orientation.z, self.pose_msg.orientation.w])
        self.a += pi/2

    def run(self):
        plt.figure(figsize=(10,10))
        while rclpy.ok:
            while self.laser == None or self.x == None or self.y == None or self.a == None:
                rclpy.spin_once(self)
            distancias = np.array(self.laser)
            distancias = np.pad(distancias, (0, 180), mode='constant', constant_values=0)

            laser_angulos = np.linspace(3*pi/2, -pi/2, 360) + self.a

            ox = np.sin(laser_angulos) * distancias
            oy = np.cos(laser_angulos) * distancias

            
            plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "r-") # lines from 0,0 to the
            plt.axis("equal")
            #bottom, top = plt.ylim()  # return the current ylim
            #plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
            plt.grid(True)
            plt.pause(0.05)
            rclpy.spin_once(self)





    def __del__(self):
        self.get_logger().info('Finalizing node! Goodbye...')

def main(args=None):
    rclpy.init(args=args)
    node = MAPEADOR()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()