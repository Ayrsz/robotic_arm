import sys
import rclpy 
import matplotlib.pyplot as plt
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from arm_interfaces.srv import FollowTrajectory
import numpy as np
from math import sin, cos, sqrt
from .utils import TrajectoryPlanner


MAX_JOINT_VELOCITY = np.pi/6 # RAD/s
MAX_JOINT_ACCELERATION = np.pi/18 # RAD/s²
L1 = 10.3
L2 = 12.28
L3 = 5.2


class MinimalClientAsync ( Node ) :

    def __init__ ( self ) :
        super () . __init__ ('robot_planner')
        self.cli = self.create_client (FollowTrajectory, 'moveServer/followTrajectory_srv')
        self.trajectoryPlanner = TrajectoryPlanner(MAX_JOINT_VELOCITY, MAX_JOINT_ACCELERATION, L1, L2, L3)
        while not self.cli.wait_for_service(timeout_sec =1.0):
          self.get_logger().info ('waiting for service ...')
          self.req = FollowTrajectory.Request()
        self.get_logger().info ('Conected')
    
    def move(self, position: np.ndarray) -> None:
        """
        Input: position in task space [x,y,z,phi]
        """

        points = self.trajectoryPlanner.move(position)
        #(theta1, theta2, theta3, theta4), (theta11, theta21)
        
        jointTrajectory = JointTrajectory()
        jointTrajectoryPoints = []
       
       
        for point in points:
            jointTrajectoryPoint = JointTrajectoryPoint()
            jointTrajectoryPoint.positions.append(point[0])
            jointTrajectoryPoint.positions.append(point[1])
            jointTrajectoryPoint.positions.append(point[2])
            jointTrajectoryPoint.positions.append(point[3])
            jointTrajectoryPoints.append(jointTrajectoryPoint)
        
        jointTrajectory.points = jointTrajectoryPoints

        self.req.data = jointTrajectory
        return self.cli.call_async(self.req)



def main():
    x, y, z, fi = sys.argv[1:5]
    position_aim = np.array([x, y, z, fi], np.float32)
    
    
    rclpy.init()
    cliente = MinimalClientAsync()
    future = cliente.move(position_aim)
    rclpy.spin_until_future_complete(cliente, future)
    response = future.result()
    print(f"RESPOSTAS {response}")

    cliente.destroy_node()
    rclpy.shutdown()
    

    
if __name__ == "__main__":
    main()
             
          

       
    




