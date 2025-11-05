import sys
from rclpy.node import Node
import rclpy
from std_srvs.srv import Trigger


class PlannerClient(Node):
    def __init__(self) -> None:
        super().__init__('planner_client')

        # cliente de serviço para solicitar execução (apenas serviço)
        self.execute_cli = self.create_client(Trigger, '/execute_trajectory')
        while not self.execute_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço /execute_trajectory...')

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
    node.request_execution()

    node.destroy_node()
    rclpy.shutdown()
