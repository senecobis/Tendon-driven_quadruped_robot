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

def postprocess_angles_minimal(angles):
    for ind in range(len(angles)):
        if angles[ind] < 0:
            angles[ind] *= -1
        if angles[ind] >= 180:
            angles[ind] -= 180

    return angles

if __name__ == "__main__":
    controller = ServoController()
    leg = Leg(l1, l2, l3, 
              slack_q1=-np.pi/2, slack_q2=0, slack_q3=0,
              clockwise_q1=False, clockwise_q2=True, clockwise_q3=False
              )
    
    line = list(np.linspace(0, l1, 100))    
    for i in line:
        time.sleep(0.1)
        x = 0
        # TODO debug why it doesn't rise the feet and it fails when we try to increase the 
        # y coordinate
        y = -l2-l3 + i*l3/l2
        z = -l1 + i*20
        angles = leg.ik_pos(x, y, z)
        angles = postprocess_angles_minimal(angles)
        controller.move_front_right(angles[0], angles[1], angles[2])
        print(angles)
    
    
    
    



