# Simulação Gazebo Turtlebot3

## Explicação

A movimentação do objeto é feito através de mensagens do tipo Twist, pela utilização de velocidade linear e angular, no eixo x e z, respectivamente.

Para mover o robô até um ponto específico, é necessário perceber qual é a sua posição atual e calcular o ângulo que ele deve rotacionar em torno do próprio eixo para traçar uma trajetória retilínea até o ponto desejado. Dessa forma, podemos mover o robô até onde desejamos.

## Execução

Para executar essa simulação, é necessária a instalação prévia do ROS-Humble e do Gazebo.

Após isso, basta executar o comando:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Para criar um executável a partir do script de movimentação do robô, basta executar o comando:

```bash
chmod +x path.py
```

Posteriormente, basta executar o comando:

```bash
./path.py
```

Alternativamente, podemos também construir um pacote, através dos seguintes passos:


## Demonstração

<https://youtu.be/lnpE4Te1KGQ>
