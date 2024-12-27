# Teleop for TurtleBot

Este é um simples nó ROS 2 para controlar um TurtleBot via teclado, utilizando o serviço `kill_robot_service` para desligar o robô.

## Comandos

- **w**: Mover para frente
- **s**: Mover para trás
- **a**: Virar à esquerda
- **d**: Virar à direita
- **q**: Sair do controle

## Dependências

- ROS 2 (Jazzy ou versão compatível)
- Pacotes ROS:
  - `rclpy`
  - `geometry_msgs`
  - `std_srvs`

## Instalação

1. Instale as dependências do ROS 2:
    ```bash
    sudo apt update
    sudo apt install ros-<distro>-rclpy ros-<distro>-geometry-msgs ros-<distro>-std-srvs
    ```

2. Instale as dependências do Python:
    ```bash
    pip install -r requirements.txt
    ```

3. Execute o nó:
    ```bash
    ros2 run <your_package> teleop_keys
    ```

## Autor

- [Seu nome]
