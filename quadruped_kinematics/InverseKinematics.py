import numpy as np
import matplotlib.pyplot as plt
import math as m
from math import atan2, sqrt, sin, cos

# LEFT_BACK_UPPER = 0
# LEFT_BACK_LOWER = 1
# LEFT_BACK_FEET = 2

# RIGHT_BACK_UPPER = 0
# RIGHT_BACK_LOWER = 0
# RIGHT_BACK_FEET = 0

# global LEN_UPPER, LEN_LOWER, LEN_FEET
LEN_UPPER = 5/100
LEN_LOWER = 15/100
LEN_FEET = 16.5/100

# RIGHT_FRONT_UPPER = 0
# RIGHT_FRONT_LOWER = 0
# RIGHT_FRONT_FEET = 0


class InverseKinematics():
    def __init__(self) -> None:
        pass
    
    def ikine(self, x4,y4,z4,l1=LEN_UPPER,l2=LEN_LOWER,l3=LEN_FEET,legs12=True):
        '''Use inverse kinematics fo calculate the leg angles for a leg to achieve a desired
        leg end point position (x4,y4,z4)

        Args:
            x4: x position of leg end point relative to leg start point coordinate system.
            y4: y position of leg end point relative to leg start point coordinate system.
            z4: z position of leg end point relative to leg start point coordinate system.
            l1: leg link 1 length
            l2: leg link 2 length
            l3: leg link 3 length
            legs12: Optional input, boolean indicating whether equations are for legs 1 or 2. 
                    If false, then equation for legs 3 and 4 is used

        Returns:
            A length 3 tuple of leg angles in the order (q1,q2,q3)
        '''

        # Supporting variable D
        D = (x4**2 + y4**2 + z4**2 - l1**2 - l2**2 - l3**2)/(2*l2*l3)

        if legs12 == True:
            q3 = atan2(sqrt(1-D**2),D)
        else:
            q3 = atan2(-sqrt(1-D**2),D)
        
        q2 = atan2(z4, sqrt(x4**2 + y4**2 - l1**2)) - atan2(l3*sin(q3), l2 + l3*cos(q3) )  

        # After using the equations, there seem to be two errors:
        #   1. The first y4 should not have a negative sign
        #   2. The entire equation should be multiplied by -1
        # The equation for q1 below reflects these changes 
        q1 = atan2(y4, x4) + atan2(sqrt(x4**2 + y4**2 - l1**2), -l1)

        return (q1,q2,q3)

    
    def invkine_leg(self, x, y, z, legs12=False):
        """inverse kinematics as described in the paper 
        "Inverse Kinematic Analysis of a Quadruped Robot", Sen, Muhammed and Bakircioglu, Veli and Kalyoncu, Mete. 

        Args:
            x (_type_): _description_
            y (_type_): _description_
            z (_type_): _description_

        Returns:
            _type_: _description_
        """
        # After using the equations, there seem to be two errors:
        #   1. The first y4 should not have a negative sign
        #   2. The entire equation should be multiplied by -1
        # The equation for q1 below reflects these changes 
        theta1 = + m.atan2(y, x) + m.atan2(m.sqrt(x**2 + y**2 - LEN_UPPER**2), -LEN_UPPER)
        
        D = (x**2 + y**2 + z**2 - LEN_UPPER**2 - LEN_LOWER**2 - LEN_FEET**2)/(2*LEN_LOWER*LEN_FEET)
        if legs12 == True:
            theta3 = m.atan2(m.sqrt(1-D**2),D)
        else:
            theta3 = m.atan2(-m.sqrt(1-D**2),D)
            
        theta2 = m.atan2(z, m.sqrt(x**2 + y**2 - LEN_UPPER**2)) - m.atan2(LEN_FEET*m.sin(theta3), LEN_LOWER + LEN_FEET*m.cos(theta3) )  
            
        return (theta1,theta2,theta3)
