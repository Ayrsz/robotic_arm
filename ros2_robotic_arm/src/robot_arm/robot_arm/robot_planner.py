import sys
import numpy as np
from rclpy.node import Node
import rclpy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .utils import TrajectoryPlanner


class PlannerClient(Node):
    def __init__(self) -> None:
        super().__init__('planner_client')

        # publisher com a trajetória planejada
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/planned_trajectory',
            10,
        )

        # cliente de serviço para solicitar execução
        self.execute_cli = self.create_client(Trigger, '/execute_trajectory')
        while not self.execute_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço /execute_trajectory...')

        # planejador de trajetória (parâmetros de exemplo; ajuste conforme seu robô)
        # velocidades em rad/s e aceleração em rad/s^2; comprimentos em metros
        self.planner = TrajectoryPlanner(
            vel_max_joint=1.0,
            aceleration_max_joint=2.0,
            l1=0.10,
            l2=0.10,
            l3=0.08,
            frequency=50,
        )

    def plan_and_publish(self, target_task_pose: np.ndarray) -> None:
        # Gera a matriz de trajetória [N x 4]
        trajectory = self.planner.move(target_task_pose, updateCurrPos=False)

        # Constrói mensagem JointTrajectory
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        dt = 1.0 / self.planner.frequency
        time_from_start = 0.0
        for row in trajectory:
            point = JointTrajectoryPoint()
            point.positions = row.tolist()
            time_from_start += dt
            # rclpy usa builtin_interfaces/Duration
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int(
                (time_from_start - int(time_from_start)) * 1e9)
            msg.points.append(point)

        self.get_logger().info(
            f'Publicando trajetória com {len(msg.points)} pontos em /planned_trajectory'
        )
        self.trajectory_pub.publish(msg)

    def request_execution(self) -> None:
        req = Trigger.Request()
        future = self.execute_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            res = future.result()
            if res.success:
                self.get_logger().info('Execução aceita pelo controlador.')
            else:
                self.get_logger().warn(f'Execução rejeitada: {res.message}')
        else:
            self.get_logger().error('Falha ao chamar /execute_trajectory')


def main() -> None:
    rclpy.init(args=sys.argv)
    node = PlannerClient()

    # Exemplo de alvo no espaço de tarefa [x, y, z, phi]
    # Ajuste conforme seus limites e workspace
    target = np.array([0.15, 0.0, 0.10, 0.0])
    node.plan_and_publish(target)
    node.request_execution()

    node.destroy_node()
    rclpy.shutdown()
