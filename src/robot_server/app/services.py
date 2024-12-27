import asyncio
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler
from math import pi
import rclpy

def pose_creator(navigator, x, y, z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, z)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose



def move_robot(positions):
    """
    Move o robô para uma lista de posições de forma assíncrona.

    Args:
        positions (list): Lista de dicionários contendo 'x', 'y' e 'orientation'.

    Exemplo:
        positions = [
            {"x": 1.0, "y": 1.0, "orientation": 0.0},
            {"x": 2.0, "y": -1.0, "orientation": 1.57},
        ]
    """
    rclpy.init()
    navigator = BasicNavigator()

    try:
        for position in positions:
            x, y, orientation = position["x"], position["y"], position["orientation"]

            goal_pose = pose_creator(navigator, x, y, orientation)
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                print("Esperando o robô chegar à posição...")
                pass

            print(f"Robô chegou à posição ({x}, {y}) com orientação {orientation}")

    except Exception as e:
        print(f"Erro durante o movimento: {e}")
    finally:
        rclpy.shutdown()

async def get_robot_position():
    """Configura os parâmetros do AMCL e retorna a posição atual do robô."""
    rclpy.init()

    node = rclpy.create_node("get_robot_position_node")

    try:
        pose = None

        node.set_parameters([
            rclpy.parameter.Parameter("/amcl/update_min_a", rclpy.Parameter.Type.DOUBLE, 0.1),
            rclpy.parameter.Parameter("/amcl/update_min_d", rclpy.Parameter.Type.DOUBLE, 0.1),
        ])

        def amcl_pose_callback(msg):
            nonlocal pose
            pose = msg
            node.get_logger().info(f"Pose recebida: {pose}")
            node.destroy_subscription(subscriber)

        subscriber = node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", amcl_pose_callback, 10)

        while pose is None:
            print("Esperando por pose...")
            rclpy.spin_once(node, timeout_sec=0.1)

        return {
            "position": {"x": pose.translation.x, "y": pose.translation.y},
            "orientation": {
                "x": pose.pose.pose.position.x,
                "y": pose.pose.pose.rotation.y,
                "z": pose.pose.pose.rotation.z,
                "w": pose.pose.pose.rotation.w,
            },
        }

    finally:
        node.destroy_node()
        rclpy.shutdown()
