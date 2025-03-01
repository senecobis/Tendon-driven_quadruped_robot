import numpy as np
from math import pi
import matplotlib.pyplot as plt

from quadruped_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
from quadruped_kinematics.utilities import transformations
from quadruped_kinematics.utilities import spot_micro_kinematics as smk


# Constants
d2r = pi/180
r2d = 180/pi

def t_rot_to_ht(t, rot):
    phi, psi, theta = rot
    x,y,z = t
    return transformations.homog_transform(phi,psi,theta, x,y,z)

def calibrate_legs():
    # Set up your specific robot's rotation conventions
    smk.set_rotation_sign_convention('right_back', 1, -1, 1)   # Example: 2nd joint inverted
    smk.set_rotation_sign_convention('right_front', 1, -1, 1)  # Example: 2nd joint inverted
    smk.set_rotation_sign_convention('left_front', -1, 1, -1)   # Example: 1st and 3rd joints inverted
    smk.set_rotation_sign_convention('left_back', -1, 1, -1)    # Example: 1st and 3rd joints inverted

    # Now all the kinematics functions will automatically handle the sign conversions
    encoder_angles = (0.5, -0.2, 0.8)  # Values from encoders
    pose = smk.t_0_to_4(*encoder_angles, 0.05, 0.12, 0.12, leg_name='right_front')

    # The returned angles will also follow your encoder conventions
    target_position = (0.2, 0.0, -0.15)
    target_angles = smk.ikine(*target_position, 0.05, 0.12, 0.12, True, leg_name='right_front')
    

if __name__ == "__main__":
    # starting coords
    x0=0
    y0=0.15
    z0=0
    phi0=0*d2r
    theta0=0*d2r
    psi0=0*d2r

    calibrate_legs()
    
    # Instantiate spot micro stick figure obeject
    ht_start = t_rot_to_ht([x0,y0,z0], [phi0, psi0, theta0])
    sm = SpotMicroStickFigure(x=x0,y=y0,z=z0,phi=phi0,theta=theta0,psi=psi0)
    sm.set_absolute_body_pose(ht_start)
    angles0 = sm.get_leg_angles_deg() 
    
