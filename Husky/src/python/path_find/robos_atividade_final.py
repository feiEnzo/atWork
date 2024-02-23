from re import T
from turtle import distance
import rclpy
import time
import numpy
import math
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import wave_front
import draw

qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)


class NavigationControl(Node):

    def __init__(self):
        super().__init__('aula_3')

        self.ranges = None
        self.create_subscription(LaserScan, '/laserscan', self.listener_callback_laser, qos_profile)

        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 0)

        self.ir_para_frente = Twist(linear=Vector3(x=1.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.ir_para_tras = Twist(linear=Vector3(x=-1.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.girar_direita = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.girar_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.curva_direita = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.curva_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))

    def sleep(self, max_seconds):
        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start
            rclpy.spin_once(self)

    def listener_callback_laser(self, msg):
        self.ranges = msg.ranges
        # self.get_logger().info(str(self.ranges))

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
        # self.get_logger().info(str(self.pose))

    def angulo(self, obj):
        return math.atan2(obj[1] - self.pose.position.y + 9, obj[0] - self.pose.position.x + 9)

    def start_control(self):
        self.get_logger().info('Inicializando o nó.')
        self.sleep(1)

        # define o mapa
        mapa = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
        ]
        # define o inicio
        turtle_inicio = [0, 0]
        # define o fim
        turtle_final = [len(mapa) - 1, len(mapa[0]) - 1]

        # utiliza o metodo wavefront para gerar o caminho que deve ser seguido
        caminho = wave_front.solve(mapa, turtle_inicio, turtle_final)

        # desenha o caminho me um mapa simplificado
        #display = draw.desenha_mundo(mapa, turtle_final)
        #draw.tartaruga(turtle_inicio, caminho, display)

        #
        while rclpy.ok:
            for x_robo_objetivo, y_robo_objetivo in caminho:
                while True:
                    rclpy.spin_once(self)
                    rot_q = self.pose.orientation
                    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

                    robo_x = abs(self.pose.position.x - 9)
                    robo_y = abs(self.pose.position.y - 9)

                    x = x_robo_objetivo - robo_x
                    y = y_robo_objetivo - robo_y

                    angle_to_goal = math.atan2(y, x)

                    if abs(angle_to_goal - theta) > 0.1:
                        self.get_logger().info("arrumando")
                        cmd = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= angle_to_goal - theta))
                        self.pub_cmd_vel.publish(cmd)
                    elif x > 0.2 and y > 0.2:
                        self.get_logger().info("andando")
                        cmd = Twist()
                        cmd.linear.x = 0.5
                        cmd.angular.z = 0.0
                        self.pub_cmd_vel.publish(cmd)
                    else:
                        self.get_logger().info("Fim")
                        cmd = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
                        break

        self.get_logger().info('Finalizando o nó.')
        self.sleep(1)

    def main(self, args=None):
        rclpy.init(args=args)
        node = NavigationControl()

        try:
            node.start_control()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    if __name__ == '__main__':
        start_control()
