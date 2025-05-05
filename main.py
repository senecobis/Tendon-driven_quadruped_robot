import time
import numpy as np
from math import pi
import matplotlib.pyplot as plt

from raspberry_controller.servo_controller import ServoController
from kinematics.forward_kinematics import Leg, t_0_to_3

# leg measure dimensions
l1 = 0.035
l2 = 0.150
l3 = 0.165

#TODO correction terms calcualated after leg assembly
def postprocess_angles_front_left(angles):
    # Post-process the output of the inverse kinematics
    postprocessed_angles = []
    for angle in angles:
        if angle < 0:
            angle *= -1
        if angle > 180:
            angle = 360 - angle
        postprocessed_angles.append(angle)
    postprocessed_angles[-1] = postprocessed_angles[-1] -40
    postprocessed_angles[0] = postprocessed_angles[0] + 90
    return np.array(postprocessed_angles)
        
if __name__ == "__main__":
    controller = ServoController()
    leg = Leg(l1, l2, l3)
    
    line = list(np.linspace(0, 10, 100))
    # for i in line:
    #     x = 0
    #     # y = -l2-l3 +(l2+l3)*i
    #     y = -l2-l3*0.5
    #     z = l1 -l1*i
    #     angles = leg.ik_pos(x, y, z)
    #     angles_ = postprocess_angles_front_left(angles)
    #     controller.move_front_left(angles_[0], angles_[1], angles_[2])
    #     time.sleep(0.1)
    #     print(f"angles: {angles_}")
    
    for i in line:
        x = 0
        # y = -l2-l3 +(l2+l3)*i
        y = -l2-l3
        z = 0 + i
        angles = leg.ik_pos(x, y, z)
        angles_ = postprocess_angles_front_left(angles)
        controller.move_front_right(angles_[0], angles_[1], angles_[2])
        time.sleep(0.1)
        print(f"angles: {angles_}")
    
    
    
    



