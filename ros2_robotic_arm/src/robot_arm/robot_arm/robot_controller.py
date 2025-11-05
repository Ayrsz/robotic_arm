import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from robot_arm_interfaces.srv import ExecuteTrajectory


class ControllerServer(Node):
    def __init__(self) -> None:
        super().__init__('controller_server')
        self._last_trajectory: Optional[JointTrajectory] = None

        # subscriber para receber a trajetória
        self.create_subscription(
            JointTrajectory,
            '/planned_trajectory',
            self._on_trajectory,
            10,
        )

        # serviço para iniciar execução recebendo a trajetória
        self.create_service(ExecuteTrajectory, '/execute_trajectory', self._on_execute)

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        self._last_trajectory = msg
        self.get_logger().info(
            f'Trajetória recebida: {len(msg.points)} pontos, joints={msg.joint_names}'
        )

    def _on_execute(self, request: ExecuteTrajectory.Request, response: ExecuteTrajectory.Response) -> ExecuteTrajectory.Response:
        trajectory = request.trajectory
        if trajectory is None or len(trajectory.points) == 0:
            response.success = False
            response.message = 'Trajetória vazia ou ausente.'
            return response

        # Aqui executaria de fato a trajetória (GPIO/servos/RPi)
        # Por enquanto, apenas confirma o recebimento e imprime no terminal
        response.success = True
        response.message = f'Executando {len(trajectory.points)} pontos.'
        self.get_logger().info(response.message)
        return response


def main() -> None:
    rclpy.init(args=sys.argv)
    node = ControllerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
