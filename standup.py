import os
import sys
import numpy as np
from math import pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from quadruped_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
from quadruped_kinematics.utilities import transformations


# Constants
d2r = pi/180
r2d = 180/pi


def t_rot_to_ht(t, rot):
    phi, psi, theta = rot
    x,y,z = t
    return transformations.homog_transform(phi,psi,theta, x,y,z)

if __name__ == "__main__":
    # starting coords
    x0=0
    y0=0.15
    z0=0
    phi0=0*d2r
    theta0=0*d2r
    psi0=0*d2r

    # Instantiate spot micro stick figure obeject
    ht_start = t_rot_to_ht([x0,y0,z0], [phi0, psi0, theta0])

    # command the actuators
    actuate = False
    if actuate:
        from raspberry_controller.servo_controller import ServoController
        controller = ServoController()
    sm = SpotMicroStickFigure(x=x0,y=y0,z=z0,phi=phi0,theta=theta0,psi=psi0)

    angles0 = sm.get_leg_angles_deg() 
    if actuate:
        controller.set_angles(*angles0)

    for slack in range(0, 10):
        x1=x0
        y1=y0+slack//1000
        z1=z0
        ht = t_rot_to_ht([x1,y1,z1], [phi0, psi0, theta0])
        sm.set_absolute_body_pose(ht)

        angles = sm.get_leg_angles_deg()
        if actuate:
            controller.set_angles(*angles)
        print(angles)


