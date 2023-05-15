#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import networkx as nx
from networkx.algorithms import approximation as approx
from math import *
from collections import deque

# Função para encontrar o melhor caminho passando por todos os nós
def best_path_all_nodes(graph, source):
    # Encontrar um ciclo hamiltoniano aproximado usando o algoritmo Christofides
    cycle = approx.traveling_salesman_problem(graph)
    # Construir o caminho começando no nó de origem
    path = [source]
    # Adicionar os nós do ciclo após o nó de origem na ordem em que aparecem no ciclo
    for i in range (len(cycle)-cycle.index(source)):
        path.append(cycle[cycle.index(source)+i])
    # Adicionar os nós do ciclo antes do nó de origem na ordem em que aparecem no ciclo
    for i in range (cycle.index(source)+1):
        path.append(cycle[i])
    # Remover qualquer nó duplicado no caminho
    repeated = []
    for i in range (1,len(path)):
        if path[i-1] == path[i]:
            repeated.append(i-1)
    for i in repeated:
        path.pop(i)
    # Transformar a lista em uma deque para otimizar as operações de remoção e adição de elementos
    path = deque(path)
    # Retornar o caminho
    return path

class TurtleController(Node):
    currentPose = []
    angleSet = False
    def __init__(self,path):
        self.path = path
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(
            msg_type=Twist, 
            topic='cmd_vel',
            qos_profile=10
        )
        self.pose_subscription = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.pose_callback,
            qos_profile=10
        )
        self.timer_ = self.create_timer(0.1, self.move_turtle)
        self.twist_msg_ = Twist()
    def move_turtle(self):
        nextPose = self.path[0]
        self.pose_subscription.callback
        dx = self.currentPose[0]-nextPose[0]
        dy = self.currentPose[1]-nextPose[1]
        ang = atan2(dy,dx)-self.currentPose[2]
        direction = ang/abs(ang)
        if self.angleSet == False:
            if abs(dx)<0.1 or abs(dy)<0.1:
                self.path.pop(0)
                return
            if abs(ang) > 0.01:
                self.twist_msg_.linear.x = 0.0
                self.twist_msg_.angular.z = 0.1*direction
            else:
                self.twist_msg_.angular.z = 0.0
                self.angleSet = True
        if self.angleSet:
            if abs(dx)>0.1 or abs(dy)>0.1:
                self.twist_msg_.linear.x = -0.1
            else:
                self.twist_msg_.linear.x = 0.0
                self.path.pop(0)
                self.angleSet = False
        self.publisher_.publish(self.twist_msg_)
    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        ang = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        self.get_logger().info(f"x={x}, y={y}, theta={theta}")
        self.currentPose = [x,y,theta]

def main(args=None):
    # Definir os nós e as arestas do grafo
    nodes = [1,2,3]
    WE = [(1,2,1),(1,3,1),(2,4,1)]
    # Criar o grafo
    G = nx.Graph()
    G.add_nodes_from(nodes)
    G.add_weighted_edges_from(WE)
    path = best_path_all_nodes(G, nodes[0])
    rclpy.init()
    turtle_controller = TurtleController(path)
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()