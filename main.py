import time
import numpy as np
from math import pi
import matplotlib.pyplot as plt

from raspberry_controller.servo_controller import ServoController
from kinematics.forward_kinematics import Leg
from kinematics.leg_kinematics import LegKinematics

# leg measure dimensions
l1 = 0.035
l2 = 0.150
l3 = 0.165

#TODO correction terms calcualated after leg assembly
def remap_angles_left(angles):
    for ind in range(len(angles)):
        if angles[ind] < 0:
            angles[ind] *= -1
        if angles[ind] > 180:
            angles[ind] = 360 - angles[ind]
    return angles

def remap_angles_right(angles):
    for ind in range(len(angles)):
        if angles[ind] < 0:
            angles[ind] *= -1
        if angles[ind] > 180:
            angles[ind] -= 180
    return angles

def test_front_right_leg(x_start=0, y_start=-l2-l3, z_start=-l1):
    # Test the front right leg
    controller = ServoController()
    leg = Leg(l1, l2, l3, 
              slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
              clockwise_q1=False, clockwise_q2=True, clockwise_q3=False
              )
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = x_start
        y = y_start + i*l3/l2
        z = z_start + i*20
        angles = leg.ik_pos(x, y, z)
        angles = remap_angles_right(angles)
        controller.move_front_right(angles[0], angles[1], angles[2])
        print(angles)

def test_lower_right_leg(x_start=0, y_start=-l2-l3, z_start=-l1):
    # Test the lower right leg
    controller = ServoController()
    leg = Leg(l1, l2, l3, 
              slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
              clockwise_q1=True, clockwise_q2=True, clockwise_q3=False
              )
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = x_start
        y = y_start + i*l3/l2
        z = z_start + i*20
        angles = leg.ik_pos(x, y, z)
        angles = remap_angles_right(angles)
        controller.move_lower_right(angles[0], angles[1], angles[2])
        print(angles)
        
def test_right_legs(x_start=0, y_start=-l2-l3, z_start=-l1):
    # Test the right legs
    controller = ServoController()
    leg_back = Leg(l1, l2, l3, 
                slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
                clockwise_q1=True, clockwise_q2=True, clockwise_q3=False
                )
    leg_front = Leg(l1, l2, l3,
                slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
                clockwise_q1=False, clockwise_q2=True, clockwise_q3=False
                )
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = x_start
        y = y_start + i*l3/l2
        z = z_start + i*20
        angles = leg_back.ik_pos(x, y, z)
        angles = remap_angles_right(angles)
        controller.move_lower_right(angles[0], angles[1], angles[2])
        
        angles = leg_front.ik_pos(x, y, z)
        angles = remap_angles_right(angles)
        controller.move_front_right(angles[0], angles[1], angles[2])
        print(angles)

def test_lower_left_leg(x_start=0, y_start=-l2-l3, z_start=-l1):
    # Test the lower left leg
    controller = ServoController()
    leg = Leg(l1, l2, l3, 
              slack_q1=-np.pi/2, slack_q2=-np.pi, slack_q3=-np.pi/2,
              clockwise_q1=True, clockwise_q2=True, clockwise_q3=False
              )
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = x_start
        y = y_start + i*l3/l2
        z = z_start + i*20
        angles = leg.ik_pos(x, y, z)
        print(f"original angles: {angles}, postprocessed angles: {remap_angles_left(angles)}")
        angles = remap_angles_left(angles)
        controller.move_lower_left(angles[0], angles[1], angles[2])
        
def test_lower_left_leg_(x_start=0, y_start=-l2-l3, z_start=-l1):
    # Test the lower left leg
    controller = ServoController()
    leg_kine = LegKinematics(l1, l2, l3)
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = x_start
        y = y_start + i*l3/l2
        z = z_start + i*20
        angles = leg_kine.lower_left_angles(x, y, z)
        controller.move_lower_left(angles[0], angles[1], angles[2])
        print(angles)
        
def test_front_left_leg(x_start=0, y_start=-l2-l3, z_start=-l1):
    # Test the front left leg
    controller = ServoController()
    leg_kine = LegKinematics(l1, l2, l3)
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = x_start
        y = y_start + i*l3/l2
        z = z_start + i*20
        angles = leg_kine.front_left_angles(x, y, z)
        controller.move_front_left(angles[0], angles[1], angles[2])
        print(angles)

if __name__ == "__main__":
    # test_front_right_leg()
    # test_lower_right_leg()
    # test_right_legs()
    # test_lower_left_leg_()
    # test_front_left_leg()
    controller = ServoController()
    controller.move_front_left(90,180,90)
    controller.move_front_right(90,0,0)
    controller.move_lower_left(90,180,90)
    controller.move_lower_right(90,0,0)

    
    
    
    



