#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odometry
from tf_transformations import euler_from_quaternion
import math


class TurtleController(Node):    
    def __init__(self, control_period=0.05):
        super().__init__('turtlecontroller')
        self.precision = 0.1 #precisão do robô
        self.odometry = Odometry()
        self.position = [2.0, 2.0, 1.0, 0.0] #array de posições
        self.index = 0 #index que vamos utilizar na lista
        self.contador = 0 #número de pontos percorridos
        self.set_point_x = self.position[self.index] #ponto x que o robô deve ir
        self.set_point_y = self.position[self.index + 1] #ponto y que o robô deve ir
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) #publisher para o tópico cmd_vel
        self.subscription = self.create_subscription(Odometry, '/odom', self.pose_callback, 10) #subscriber para o tópico odom
        self.control_timer = self.create_timer(timer_period_sec = control_period, callback = self.control_callback) #timer para o controle

    def pose_callback(self, msg):
        position_x = msg.pose.pose.position.x #posição x do robô
        position_y = msg.pose.pose.position.y #posição y do robô
        angle = msg.pose.pose.orientation #ângulo do robô
        _, _, self.theta = euler_from_quaternion([angle.x, angle.y, angle.z, angle.w])
        self.actual_pose_x = position_x #posição x atual do robô
        self.actual_pose_y = position_y #posição y atual do robô

    def control_callback(self):
        msg = Twist()
        self.set_point_x = self.position[self.index]
        self.set_point_y = self.position[self.index + 1]
        x_difference = self.set_point_x - self.actual_pose_x #diferença entre o ponto x que o robô deve ir e o ponto x atual do robô
        y_difference = self.set_point_y - self.actual_pose_y #diferença entre o ponto y que o robô deve ir e o ponto y atual do robô
        angle = math.atan2(y_difference, x_difference)
        theta_difference = angle - self.theta #diferença entre o ângulo que o robô deve estar e o ângulo atual do robô
        if abs(x_difference) <= self.precision and abs(y_difference) <= self.precision: 
            if self.contador == 1: #se o robô já passou por todos os pontos
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                exit()
            else:
                self.index = 2 #atualiza o index
                self.contador = 1 #incrementa o contador
        if abs(theta_difference) >= (self.precision -0.05):
            msg.linear.x = 0.0
            if theta_difference > 0:
                msg.angular.z = 0.2
            else:
                msg.angular.z = -0.2
        elif abs(x_difference) >= self.precision:
            msg.linear.x = 0.2
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtle = TurtleController()
    rclpy.spin(turtle)
    turtle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()