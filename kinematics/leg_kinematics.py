import numpy as np
from kinematics.forward_kinematics import Leg

class LegKinematics:
    def __init__(self, l1, l2, l3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.lower_right = Leg(l1, l2, l3, 
                            slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
                            clockwise_q1=True, clockwise_q2=True, clockwise_q3=False
                            )
        self.lower_left = Leg(l1, l2, l3,
                            slack_q1=-np.pi/2, slack_q2=-np.pi, slack_q3=-np.pi/2,
                            clockwise_q1=True, clockwise_q2=True, clockwise_q3=False
                            )
        self.front_right = Leg(l1, l2, l3,
                            slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
                            clockwise_q1=False, clockwise_q2=True, clockwise_q3=False
                            ) # V
        self.front_left = Leg(l1, l2, l3,
                            slack_q1=-np.pi/2, slack_q2=-np.pi, slack_q3=-np.pi/2,
                            clockwise_q1=False, clockwise_q2=True, clockwise_q3=False
                            )
    @staticmethod
    def remap_angles_right(angles):
        """Remap angles for right leg to be in the range [0, 180]"""
        for ind in range(len(angles)):
            if angles[ind] < 0:
                angles[ind] *= -1
            if angles[ind] >= 180:
                angles[ind] -= 180
        return angles

    @staticmethod
    def remap_angles_left(angles):
        for ind in range(len(angles)):
            if angles[ind] < 0:
                angles[ind] *= -1
            if angles[ind] > 180:
                angles[ind] = 360 - angles[ind]
        return angles
    
    def lower_right_angles(self, x,y,z):
        """Get angles for lower right leg"""
        angles = self.lower_right.ik_pos(x, y, z)
        angles = self.remap_angles_right(angles)
        return angles
    
    def front_right_angles(self, x,y,z):
        """Get angles for front right leg"""
        angles = self.front_right.ik_pos(x, y, z)
        angles = self.remap_angles_right(angles)
        return angles
    
    def lower_left_angles(self, x,y,z):
        """Get angles for lower left leg"""
        angles = self.lower_left.ik_pos(x, y, z)
        angles = self.remap_angles_left(angles)
        return angles
    
    def front_left_angles(self, x,y,z):
        """Get angles for front left leg"""
        angles = self.front_left.ik_pos(x, y, z)
        angles = self.remap_angles_left(angles)
        return angles