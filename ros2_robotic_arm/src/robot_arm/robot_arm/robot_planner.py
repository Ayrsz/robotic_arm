import sys
import numpy as np
from rclpy.node import Node
import rclpy
from robot_arm_interfaces.srv import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .utils import TrajectoryPlanner


class PlannerClient(Node):
    def __init__(self) -> None:
        super().__init__('planner_client')

        # cliente de serviço para enviar a trajetória
        self.execute_cli = self.create_client(
            ExecuteTrajectory, '/execute_trajectory')
        while not self.execute_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço /execute_trajectory...')

        # planejador para gerar a trajetória antes de enviar
        self.planner = TrajectoryPlanner(
            vel_max_joint=1.0,
            aceleration_max_joint=2.0,
            l1=0.10,
            l2=0.10,
            l3=0.08,
            frequency=50,
        )

    def request_execution(self) -> None:
        # exemplo: gera uma trajetória e envia via serviço
        target = np.array([0.15, 0.0, 0.10, 0.0])
        traj_matrix = self.planner.move(target, updateCurrPos=False)

        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        dt = 1.0 / self.planner.frequency
        time_from_start = 0.0
        for row in traj_matrix:
            pt = JointTrajectoryPoint()
            pt.positions = row.tolist()
            time_from_start += dt
            pt.time_from_start.sec = int(time_from_start)
            pt.time_from_start.nanosec = int(
                (time_from_start - int(time_from_start)) * 1e9)
            msg.points.append(pt)

        req = ExecuteTrajectory.Request()
        req.trajectory = msg
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
    node.request_execution()

    node.destroy_node()
    rclpy.shutdown()
