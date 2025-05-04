import time
from adafruit_servokit import ServoKit

class ServoController:
    def __init__(self):
        kit = ServoKit(channels=16)

        self.left_back_upper = kit.servo[0]
        self.left_back_lower = kit.servo[1]
        self.left_back_feet = kit.servo[2]
        self.right_back_upper = kit.servo[3]
        self.right_back_lower = kit.servo[4]
        self.right_back_feet = kit.servo[5]
        self.left_front_upper = kit.servo[6]
        self.left_front_lower = kit.servo[7]
        self.left_front_feet = kit.servo[8]
        self.right_front_upper = kit.servo[9]
        self.right_front_lower = kit.servo[10]
        self.right_front_feet = kit.servo[11]
        
    def move_lower_left(self, q1, q2, q3):
        # move the left back leg
        self.left_back_upper.angle = q1
        self.left_back_lower.angle = q2
        self.set_constrained_feet_pos(q3, self.left_back_feet)
        
    def move_lower_right(self, q1, q2, q3):
        # move the right back leg
        self.right_back_upper.angle = q1
        self.right_back_lower.angle = q2
        self.set_constrained_feet_pos(q3, self.right_back_feet)
        
    def move_front_left(self, q1, q2, q3):
        # move the left front leg
        self.left_front_upper.angle = q1
        self.left_front_lower.angle = q2
        self.set_constrained_feet_pos(q3, self.left_front_feet)
        
    def move_front_right(self, q1, q2, q3):
        # move the right front leg
        self.right_front_upper.angle = q1
        self.right_front_lower.angle = q2
        self.set_constrained_feet_pos(q3, self.right_front_feet)

    @staticmethod
    def set_constrained_feet_pos(pos: int, feet_: ServoKit):
        # limits for feet is 0-90 deg, this function imposes the constraint
        if pos < 0:
            pos = 0

        if pos > 90:
            pos = 90

        feet_.angle = pos

    def set_aestetic_pose(self):
        # just aestetic static pose for photos
        self.left_back_upper.angle = 90
        self.left_back_lower.angle = (180 -40)
        self.set_constrained_feet_pos(45, self.left_back_feet)

        self.right_back_upper.angle = 90
        self.right_back_lower.angle = 40
        self.set_constrained_feet_pos(45, self.right_back_feet)

        self.left_front_upper.angle = 90
        self.left_front_lower.angle = (180 - 40)
        self.set_constrained_feet_pos(45, self.left_front_feet)

        self.right_front_upper.angle = 100
        self.right_front_lower.angle = 40
        self.set_constrained_feet_pos(45, self.right_front_feet)


    def standup(self):
        # standup pose
        self.left_back_upper.angle = 90
        self.left_back_lower.angle = 90
        self.set_constrained_feet_pos(0, self.left_back_feet)

        self.right_back_upper.angle = 90
        self.right_back_lower.angle = 90
        self.set_constrained_feet_pos(0, self.right_back_feet)

        self.left_front_upper.angle = 90
        self.left_front_lower.angle = 90
        self.set_constrained_feet_pos(0, self.left_front_feet)

        self.right_front_upper.angle = 90
        self.right_front_lower.angle = 90
        self.set_constrained_feet_pos(0, self.right_front_feet)

    def sitdown(self):
        # sitdown pose
        self.left_back_upper.angle = 90
        self.left_back_lower.angle = 90
        self.set_constrained_feet_pos(0, self.left_back_feet)

        self.right_back_upper.angle = 90
        self.right_back_lower.angle = 90 # with 0 is not aligned with the other
        self.set_constrained_feet_pos(90, self.right_back_feet)

        self.left_front_upper.angle = 90
        self.left_front_lower.angle = 90
        self.set_constrained_feet_pos(0, self.left_front_feet)

        self.right_front_upper.angle = 90
        self.right_front_lower.angle = 90
        self.set_constrained_feet_pos(90, self.right_front_feet)

    
    def set_angles(self, lb, rb, lf, rf):
        lbu, lbl, lbf = lb
        rbu, rbl, rbf = rb
        lfu, lfl, lff = lf
        rfu, rfl, rff = rf
        
        self.left_back_upper.angle = lbu
        self.left_back_lower.angle = lbl
        self.set_constrained_feet_pos(lbf, self.left_back_feet)

        self.right_back_upper.angle = rbu
        self.right_back_lower.angle = rbl
        self.set_constrained_feet_pos(rbf, self.right_back_feet)

        self.left_front_upper.angle = lfu
        self.left_front_lower.angle = lfl
        self.set_constrained_feet_pos(lff, self.left_front_feet)

        self.right_front_upper.angle = rfu
        self.right_front_lower.angle = rfl
        self.set_constrained_feet_pos(rff, self.right_front_feet)
        
if __name__ == "__main__":
    controller = ServoController()
    # controller.set_aestetic_pose()
    controller.sitdown()
