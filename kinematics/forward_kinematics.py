import kinematics.transformations as transformations
from math import pi, cos, sin, atan2, sqrt
from scipy.optimize import minimize
import numpy as np

# NOTE all the angles are in radians
# Constants
d2r = pi/180
r2d = 180/pi

def t_0_to_1(q1,l1):
    '''Create the homogeneous transformation matrix for joint 0 to 1 for a quadriped leg.

    Args:
        theta1: Rotation angle in radians of the hip joint
        l1: Length of the hip joint link

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 0 to 1
    '''
    # NOTE since by q1 definition it starts from 90 deg we subtract/add 90 deg
    # q1 = q1 - pi/2
    t_01 = np.block(
        [ [ transformations.rotz(q1), np.array([[0],[0],[-l1]]) ],
                                    [np.array([0,0,0,1])] 
        ]    
        )
    return t_01

def t_1_to_2(q2, l2):
    '''Create the homogeneous transformation matrix for joint 1 to 2 for a quadriped leg.

    Args:
        None

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 1 to 2
    '''
    t_12 = np.block(
        [ [ transformations.rotx(q2), np.array([[0],[-l2*cos(q2)],[-l2*sin(q2)]]) ],
                                    [np.array([0,0,0,1])] 
        ]    
        )
    return t_12

def t_2_to_3(q3, l3):
    '''Create the homogeneous transformation matrix for joint 1 to 2 for a quadriped leg.

    Args:
        theta2: Rotation angle in radians of the leg joint
        l2: Length of the upper leg link

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 2 to 3
    '''
    t_23 = np.block(
        [ [ transformations.rotx(q3), np.array([[0],[-l3*cos(q3)],[l3*sin(q3)]]) ],
                                    [np.array([0,0,0,1])] 
        ]    
        )
    return t_23

def t_0_to_2(q1, q2, l1, l2):
    return np.matmul(t_0_to_1(q1,l1), t_1_to_2(q2,l2))

def t_0_to_3(q1, q2, q3, l1, l2, l3):
    return np.matmul(np.matmul(t_0_to_1(q1,l1), t_1_to_2(q2,l2)), t_2_to_3(q3,l3))

class Leg():
    def __init__(self, l1, l2, l3, 
                 slack_q1=0, slack_q2=0, slack_q3=0, 
                 clockwise_q1=False, clockwise_q2=False, clockwise_q3=False
                 ):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        
        self.q1_guess = 0
        self.q2_guess = 0
        self.q3_guess = 0
        
        # slack angles are the angles at which the leg is assembled
        self.slack_q1 = slack_q1
        self.slack_q2 = slack_q2
        self.slack_q3 = slack_q3
        
        # clockwise angles are the directions of the joints
        # if True the joint rotates clockwise
        self.clockwise_q1 = clockwise_q1
        self.clockwise_q2 = clockwise_q2
        self.clockwise_q3 = clockwise_q3
        
        self.angle_bounds = [(-2*np.pi, 2*np.pi)] * 3  # For theta1, theta2, theta3
        # self.angle_bounds = None

    def t_0_to_1(self, q1):
        q1 = q1 + self.slack_q1
        if self.clockwise_q1:
            q1 = -q1
        return t_0_to_1(q1, self.l1)
    
    def t_0_to_2(self, q1, q2):
        q1 = q1 + self.slack_q1
        q2 = q2 + self.slack_q2
        if self.clockwise_q1:
            q1 = -q1
        if self.clockwise_q2:
            q2 = -q2
        return t_0_to_2(q1, q2, self.l1, self.l2)
    
    def t_0_to_3(self, q1, q2, q3):
        q1 = q1 + self.slack_q1
        q2 = q2 + self.slack_q2
        q3 = q3 + self.slack_q3
        if self.clockwise_q1:
            q1 = -q1
        if self.clockwise_q2:
            q2 = -q2
        if self.clockwise_q3:
            q3 = -q3
        return t_0_to_3(q1, q2, q3, self.l1, self.l2, self.l3)
    
    def ik_cost(self, q, T_target):
        # Inverse kinematics cost function
        T_est = self.t_0_to_3(q[0], q[1], q[2])
        position_error = np.linalg.norm(T_est[:3, 3] - T_target[:3, 3])
        # Optionally include orientation error
        return position_error
    
    def ik_position_cost(self, q, postion_target):
        # Inverse kinematics cost function
        T_est = self.t_0_to_3(q[0], q[1], q[2])
        postion_est = T_est[:3, -1]
        position_error = np.linalg.norm(postion_est - postion_target)
        return position_error
    
    def ik_pos(self, x, y, z):
        theta_guess = np.array([self.q1_guess, self.q2_guess, self.q3_guess])
        position_target = np.array([x, y, z])
        result = minimize(self.ik_position_cost, theta_guess, args=(position_target,), bounds=self.angle_bounds)
        angles = result.x
        
        # Assign new initial guesses
        self.q1_guess = angles[0]
        self.q2_guess = angles[1]
        self.q3_guess = angles[2]
        return (angles*r2d).astype(int)
    
    def ik_z_cost(self, q, z_target):
        T_est = self.t_0_to_3(q[0], q[1], q[2])
        z_est = T_est[2, 3]
        return (z_est - z_target)**2  # Squared error on z only
    
    def ik_z_only(self, z):
        theta_guess = np.array([self.q1_guess, self.q2_guess, self.q3_guess])
        result = minimize(self.ik_z_cost, theta_guess, args=(z,), bounds=self.angle_bounds)
        angles = result.x

        # Save for warm-starting future optimizations
        self.q1_guess, self.q2_guess, self.q3_guess = angles
        return (angles * r2d).astype(int)

    
if __name__ == "__main__":
    t_03 = t_0_to_3(q1=0, q2=0, q3=0, l1=0.1, l2=0.1, l3=0.1)