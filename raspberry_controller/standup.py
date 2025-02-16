import os
import numpy as np
from math import pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from quadruped_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
from quadruped_kinematics.utilities import transformations

from servo_controller import ServoController


# Constants
d2r = pi/180
r2d = 180/pi

# starting coords
x0=0
y0=0.15
z0=0
phi0=0*d2r
theta0=0*d2r
psi0=0*d2r

# Instantiate spot micro stick figure obeject
ht_start = transformations.homog_transform(x0,y0,z0,phi0,theta0,psi0)

# leg measure dimensions
l1 = 0.035
l2 = 0.150
l3 = 0.165
l = 0.25
w = 0.14

def t_rot_to_ht(t, rot):
    phi, psi, theta = rot
    x,y,z = t
    return transformations.homog_transform(x,y,z,phi,psi,theta)

if __name__ == "__main__":
    controller = ServoController()
    sm = SpotMicroStickFigure(x=x0,y=y0,z=z0,phi=phi0,theta=theta0,psi=psi0)

    sm.body_length = l
    sm.body_width = w
    sm.hip_length = l1
    sm.upper_leg_length = l2
    sm.lower_leg_length = l3

    angles = sm.get_leg_angles_deg()
    controller.set_angles(*angles)

    phi = 0
    psi = 0
    theta = 0
    x1=x0
    y1=y0+0.1
    z1=z0
    ht = t_rot_to_ht([x1,y1,z1], [phi, psi, theta])
    sm.set_absolute_body_pose(ht)

    angles = sm.get_leg_angles_deg()
    controller.set_angles(*angles)



