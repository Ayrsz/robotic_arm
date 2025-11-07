import numpy as np
from math import sin, cos, sqrt

# [GRAUS] -> [GRAUS]

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
        """
        Receives a position in task space and returns the joint angles to reach that position \n
        Input: p = [x, y, z, fi] (fi is the angle of the hand with the horizontal plane) \n
        Output: q = [theta1, theta2, theta3, theta4] \n
        """
        
        assert len(p) == 4 

        x, y, z, fi = p
        fi = np.deg2rad(fi) #FI IS RECIEVED IN DEGRESS
        
 
        #rotate in the z axis
        theta1 = np.arctan2(y,x)

        comprimento = (x**2 + y**2)**0.5

        arccos_factor_numerator = (comprimento-self.l3*cos(fi))**2 + (z-self.l3*sin(fi))**2 - self.l1**2 - self.l2**2
        arccos_factor_denominator = 2*self.l1*self.l2
        if(abs(arccos_factor_numerator/arccos_factor_denominator) > 1):
          theta3 = 0
          if abs(arccos_factor_numerator/arccos_factor_denominator) > 1.1:
            print("the value supposed to be in arccos is big, so the values are not gonna match")
        else:
          theta3 = -np.arccos(arccos_factor_numerator/arccos_factor_denominator) # choose the negative theta3 angle instead of the positive one


        #Theta 2 in function of theta3.
        arctan_factor1 = (z - self.l3*sin(fi))/(comprimento - self.l3*cos(fi))
        arctan_factor2 = (self.l2*sin(theta3))/(self.l1 + self.l2*cos(theta3))
        theta2 = np.arctan(arctan_factor1) - np.arctan(arctan_factor2)



        #Hand rotate
        theta4 = fi - theta2 - theta3

        return np.rad2deg(np.array([theta1, theta2, theta3, theta4])) #RETURN IN DEGRESS

    #JOINTS SPACE -> TASK SPACE
    def FK(self, q: np.ndarray) -> np.ndarray:
        """
        Receives a position in joint space and returns the position in task space \n
        Input: q = [theta1, theta2, theta3, theta4] (fi is the angle of the hand with the horizontal plane) \n
        Output: p = [x, y, z, fi] (fi is the angle of the hand with the horizontal plane) \n
        """
        q = np.deg2rad(q) #Q IS RECEIVED IN DEGREE
        # solving for Z
        theta1, theta2, theta3, theta4 = q
        z = self.l1*sin(theta2) + self.l2*sin(theta2+theta3) + self.l3*sin(theta2+theta3+theta4)

        # solving for X and Y
        comprimento = self.l1*cos(theta2) + self.l2*cos(theta2+theta3) + self.l3*cos(theta2+theta3+theta4)
        x = comprimento * cos(theta1)
        y = comprimento * sin(theta1)

        phi = theta2+theta3+theta4
        phi = np.rad2deg(phi) #PHI IS IN DEGREE
        return np.array([x,y,z,phi])


class TrajectoryPlanner:
    def __init__(self, vel_max_joint : float, aceleration_max_joint : float,
                 l1, l2 , l3, frequency = 50, current_position_joint = np.array([0, 90, -90, 0])):
    
        self.vel_max_joint = vel_max_joint
        self.aceleration_max_joint = aceleration_max_joint
        self.current_position_joint = current_position_joint 
        self.frequency = frequency
        self.kin = Kinematic(l1, l2, l3)
    
    # plan only a single joint
    def planning_joint(self, q_initial: float, q_final: float) -> np.ndarray:
        """
        Returns a trajectory of a single joint 
        Input: q_initial = initial angle of the joint 
               q_final = final angle of the joint 
        Output: positions = trajectory of a single joint
        """

        distance = q_final - q_initial # delta s
        forward = 1 if (distance >= 0) else -1  #direction of movement(+1 if distance positive, -1 if negative)
        
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

        # get amount of samples for aceleration and deceleration
        slices_in_aceleration = int(time_in_aceleration * self.frequency) + 1
        slices_in_deceleration = int(time_in_deceleration * self.frequency)

        # update the acelerations array with the aceleration and deceleration values
        acelerations[:slices_in_aceleration] = forward*self.aceleration_max_joint
        acelerations[-slices_in_deceleration:] = -1*forward*self.aceleration_max_joint

        dt = 1 / self.frequency
        for i in range(1, slices):
            speeds[i] = speeds[i-1] + acelerations[i]*dt
            positions[i] = positions[i-1] + speeds[i]*dt


        print(f"GOING FROM {q_initial} to {positions[-1]}")
        return positions

    # plan the trajectory from the current position to the target position in joint space
    def move(self, target_position_task: np.ndarray, updateCurrPos: bool = True) -> np.ndarray:
        """
        Returns a trajectory of the arm from the current position to the target position in joint space \n
        Input: target_position_task = [x, y, z, fi], fi in DEGRESS \n
        Output: trajectory = Each row = [theta1, theta2, theta3, theta4] in DEGRESS
        """

        # get target position in joints space
        
        target_position_joints = self.kin.IK(target_position_task)
        
        
        # get the trajectory for each joint
        planning_joint1 = self.planning_joint(self.current_position_joint[0], target_position_joints[0]) 
        planning_joint2 = self.planning_joint(self.current_position_joint[1], target_position_joints[1])
        planning_joint3 = self.planning_joint(self.current_position_joint[2], target_position_joints[2])
        planning_joint4 = self.planning_joint(self.current_position_joint[3], target_position_joints[3])

        # get the legth of the trajectory that takes more time to finish the movement
        max_len = max(len(planning_joint1), len(planning_joint2), len(planning_joint3), len(planning_joint4))

        #filling gaps with the last position of each joint to make all joints have the same length
        extra_array1 = np.ones(max_len - len(planning_joint1))*planning_joint1[-1]
        planning_joint1 = np.concatenate((planning_joint1, extra_array1))

        extra_array2 = np.ones(max_len - len(planning_joint2))*planning_joint2[-1]
        planning_joint2 = np.concatenate((planning_joint2, extra_array2))

        extra_array3 = np.ones(max_len - len(planning_joint3))*planning_joint3[-1]
        planning_joint3 = np.concatenate((planning_joint3, extra_array3))

        extra_array4 = np.ones(max_len - len(planning_joint4))*planning_joint4[-1]
        planning_joint4 = np.concatenate((planning_joint4, extra_array4))

        # update the current position of the arm
        if updateCurrPos:
            self.current_position_joint = np.array([planning_joint1[-1], planning_joint2[-1], planning_joint3[-1], planning_joint4[-1]])

        
        return np.stack([planning_joint1, planning_joint2, planning_joint3, planning_joint4], axis=1)