import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


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

        # serviço para iniciar execução
        self.create_service(Trigger, '/execute_trajectory', self._on_execute)

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        self._last_trajectory = msg
        self.get_logger().info(
            f'Trajetória recebida: {len(msg.points)} pontos, joints={msg.joint_names}'
        )

    def _on_execute(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._last_trajectory is None:
            response.success = False
            response.message = 'Nenhuma trajetória recebida.'
            return response

        # Aqui executaria de fato a trajetória (GPIO/servos/RPi)
        # Por enquanto, apenas confirma o recebimento e imprime no terminal
        response.success = True
        response.message = f'Executando {len(self._last_trajectory.points)} pontos.'
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
