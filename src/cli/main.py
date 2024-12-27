import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi
import typer
from yaspin import yaspin
import inquirer

poses = [
    (0.368, 0.049),
    (-0.187, 0.217),
    (-0.177, -0.159),
    (-0.769, 0.263),
    (-0.718, -0.190),
    (-0.608, -0.611),
    (-1.437, 0.066),
    (-1.219, -0.415),
    (-1.090, -0.809),
    (-0.583, -0.662)
]

app = typer.Typer()

def move_to_pose(x, y):
    rclpy.init()
    nav = BasicNavigator()

    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, pi/4)
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    
    nav.goToPose(goal_pose)
    
    with yaspin(text="Movendo o robô para a posição selecionada...", color="cyan") as spinner:
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                spinner.text = f"Distância restante: {feedback.distance_remaining:.2f} metros"
        spinner.text = "Robo chegou na posição!"
        spinner.ok("✔")
    
    rclpy.shutdown()

@app.command()
def select_pose():
    questions = [
        inquirer.List(
            "pose",
            message="Escolha a posição para mover o robô",
            choices=[f"Pose {i+1}: X={x}, Y={y}" for i, (x, y) in enumerate(poses)]
        )
    ]
    answer = inquirer.prompt(questions)
    
    if answer:
        pose_index = int(answer["pose"].split()[1][:-1]) - 1
        x, y = poses[pose_index]
        typer.echo(f"Movendo o robô para a posição selecionada: X={x}, Y={y}")
        move_to_pose(x, y)
    else:
        typer.echo("Nenhuma posição selecionada.")

if __name__ == "__main__":
    app()
