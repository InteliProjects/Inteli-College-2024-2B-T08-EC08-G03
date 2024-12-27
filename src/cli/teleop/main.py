import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# Constantes de velocidade máxima
MAX_LINEAR_SPEED = 0.21
MAX_ANGULAR_SPEED = 2.8

class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__("teleop_keys")
        
        # Publicador para o tópico cmd_vel do TurtleBot
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Aviso sobre os comandos
        self.show_instructions()

    def show_instructions(self):
        instructions = (
            "Control the robot with keys:\n"
            "  'w' - move forward (half speed)\n"
            "  's' - move backward (half speed)\n"
            "  'a' - turn left (half speed)\n"
            "  'd' - turn right (half speed)\n"
            "  'W', 'S', 'A', 'D' - move at max speed\n"
            "  'q' - quit"
        )
        print(instructions)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def publish_velocity(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)
        print(f"\rLinear speed: {linear} | Angular speed: {angular}", end="")

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def shutdown_robot(self):
        print("\nShutting down the robot...")

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                # Define as velocidades com base na tecla pressionada e se é maiúscula (máxima) ou minúscula (metade)
                if key in ["W", "S", "A", "D"]:
                    linear_speed = MAX_LINEAR_SPEED
                    angular_speed = MAX_ANGULAR_SPEED
                else:
                    linear_speed = MAX_LINEAR_SPEED / 2
                    angular_speed = MAX_ANGULAR_SPEED / 2

                if key == "q":
                    break
                elif key in ["w", "W"]:
                    self.publish_velocity(linear=linear_speed)
                elif key in ["s", "S"]:
                    self.publish_velocity(linear=-linear_speed)
                elif key in ["a", "A"]:
                    self.publish_velocity(angular=angular_speed)
                elif key in ["d", "D"]:
                    self.publish_velocity(angular=-angular_speed)
                else:
                    self.stop_robot()

        finally:
            self.shutdown_robot()


def main():
    rclpy.init()
    teleop_node = TeleopKeyNode()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
