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

#Calcular tempo de aceleração, tempo de desaceleração, fazer 15 graus por seg de velocidade max
class Kinematic:
    #theta1 -> Base rotation
    #theta2, theta3 -> Two joints adjacents to the base rotation
    #theta4 -> Hand joint
    def __init__(self, l1, l2, l3):
        #init the size of the joints
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    #TASK SPACE -> JOINTS SPACE
    def IK(self, p : np.ndarray) -> np.ndarray:

        assert len(p) == 4
        x, y, z, fi = p
        #rotate in the z axis
        theta1 = np.arctan2(y,x)

        comprimento = (x**2 + y**2)**0.5

        arccos_factor_numerator = (comprimento-self.l3*cos(fi))**2 + (z-self.l3*sin(fi))**2 - self.l1**2 - self.l2**2
        arccos_factor_denominator = 2*self.l1*self.l2
        print("O: ", arccos_factor_numerator/arccos_factor_denominator)
        if(arccos_factor_numerator/arccos_factor_denominator > 1):
          theta3 = 0
          if arccos_factor_numerator/arccos_factor_denominator > 1.2:
            print("the value supposed to be in arccos is big, so the values are not gonna match")
        else:
          theta3 = -np.arccos(arccos_factor_numerator/arccos_factor_denominator) # choose the negative theta3 angle instead of the positive one


        #Theta 2 in function of theta3.
        arctan_factor1 = (z - self.l3*sin(fi))/(comprimento - self.l3*cos(fi))
        arctan_factor2 = (self.l2*sin(theta3))/(self.l1 + self.l2*cos(theta3))
        theta2 = np.arctan(arctan_factor1) - np.arctan(arctan_factor2)



        #Hand rotate
        theta4 = fi - theta2 - theta3

        return np.array([theta1, theta2, theta3, theta4])

    def FK(self, q: np.ndarray) -> np.ndarray:
        # solving for Z
        theta1, theta2, theta3, theta4 = q
        z = self.l1*sin(theta2) + self.l2*sin(theta2+theta3) + self.l3*sin(theta2+theta3+theta4)

        # solving for X and Y
        comprimento = self.l1*cos(theta2) + self.l2*cos(theta2+theta3) + self.l3*cos(theta2+theta3+theta4)
        x = comprimento * cos(theta1)
        y = comprimento * sin(theta1)

        phi = theta2+theta3+theta4

        return np.array([x,y,z,phi])


class TrajectoryPlanner:
    def __init__(self, vel_max_joint : float, aceleration_max_joint : float,
                 l1, l2 , l3, frequency = 50, current_position_joint = np.array([0, np.pi/2, -np.pi/2, 0])):
        #INITIAL POSITION IN (X, Y, Z, FI) !!!!
        self.vel_max_joint = vel_max_joint
        self.aceleration_max_joint = aceleration_max_joint
        self.current_position_joint = current_position_joint 
        self.frequency = frequency
        self.kin = Kinematic(l1, l2, l3)
    
    def planning_joint(self, q_initial, q_final):
        #The planing will be in the JOINT space
        #better control and we need to config the veloctiy
        #and convert to the space of joints
        #the robot operate at 50Hz per second

        distance = q_final - q_initial # delta s
        forward = 1 if (distance >= 0) else -1  
        
        time_in_aceleration = self.vel_max_joint / self.aceleration_max_joint #t = v / a
        distance_in_aceleration = self.aceleration_max_joint*(time_in_aceleration**2)/2 # S = at**2/2
        time_in_deceleration = time_in_aceleration # t = v / a
        distance_in_deceleration = self.vel_max_joint*time_in_deceleration + -1*self.aceleration_max_joint*(time_in_deceleration**2)/2 # S = Vo*t - at**2/2

        if distance_in_aceleration + distance_in_deceleration < distance:
            gap_distance = distance - distance_in_aceleration - distance_in_deceleration
            time_in_vel_cte = gap_distance / self.vel_max_joint
        elif distance_in_aceleration + distance_in_deceleration == distance:
            time_in_vel_cte = 0
        elif distance_in_aceleration + distance_in_deceleration > distance:
            #d/2 = a(t**2)/2 so t = sqrt(d/a)
            time_in_aceleration = sqrt(abs(distance)/self.aceleration_max_joint)
            time_in_deceleration = time_in_aceleration
            time_in_vel_cte = 0


        total_time = time_in_aceleration + time_in_vel_cte + time_in_deceleration

        #convert the aceleration from seconds to slices

        slices = int(total_time * self.frequency) + 1

        #init the vectors
        positions = np.zeros(slices)
        positions[0] = q_initial
        speeds = np.zeros(slices)
        acelerations = np.zeros(slices)


        slices_in_aceleration = int(time_in_aceleration * self.frequency) + 1
        slices_in_deceleration = int(time_in_deceleration * self.frequency)

        acelerations[:slices_in_aceleration] = forward*self.aceleration_max_joint
        acelerations[-slices_in_deceleration:] = -1*forward*self.aceleration_max_joint

        dt = 1 / self.frequency
        for i in range(1, slices):
            speeds[i] = speeds[i-1] + acelerations[i]*dt
            positions[i] = positions[i-1] + speeds[i]*dt


        print(f"GIVEN JOINT FINAL {q_final}, GOT JOINT FINAL {positions[-1]}")
        return positions

    def planning_for_points(self, target_position_task):
        
        target_position_joints = self.kin.IK(target_position_task)
        
        planning_joint1 = self.planning_joint(self.current_position_joint[0], target_position_joints[0])
        planning_joint2 = self.planning_joint(self.current_position_joint[1], target_position_joints[1])
        planning_joint3 = self.planning_joint(self.current_position_joint[2], target_position_joints[2])
        planning_joint4 = self.planning_joint(self.current_position_joint[3], target_position_joints[3])

        max_len = max(len(planning_joint1), len(planning_joint2), len(planning_joint3), len(planning_joint4))

        extra_array1 = np.ones(max_len - len(planning_joint1))*planning_joint1[-1]
        planning_joint1 = np.concatenate((planning_joint1, extra_array1))

        extra_array2 = np.ones(max_len - len(planning_joint2))*planning_joint2[-1]
        planning_joint2 = np.concat((planning_joint2, extra_array2))

        extra_array3 = np.ones(max_len - len(planning_joint3))*planning_joint3[-1]
        planning_joint3 = np.concat((planning_joint3, extra_array3))

        extra_array4 = np.ones(max_len - len(planning_joint4))*planning_joint4[-1]
        planning_joint4 = np.concat((planning_joint4, extra_array4))
        self.current_position_joint = np.array([planning_joint1, planning_joint2, planning_joint3, planning_joint4])
        return self.current_position_joint

    

    

    
    

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
    
    




