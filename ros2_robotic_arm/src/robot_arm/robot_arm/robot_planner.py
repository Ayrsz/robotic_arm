import sys
import rclpy 
import matplotlib.pyplot as plt
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory 

import numpy as np
from math import sin, cos, sqrt


def main():
    print('Hi from planner.')

if __name__ == '__main__':
    main()


class MinimalClientAsync ( Node ) :

    def __init__ ( self ) :
        super () . __init__ ('minimal_client_async')
        self.cli = self.create_client (JointTrajectory, 'robot_planner')
        while not self.cli.wait_for_service(timeout_sec =1.0):
          self.get_logger().info ('waiting for service ...')
          self.req = JointTrajectory
    def send_request (self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)
    
    




