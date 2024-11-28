import argparse
import time
from robot  import *
from calibrate_workspace import *
from motion_planner import *
from utils  import *


def main():
    fa = FrankaArm()
    robot = Robot()

    fa.reset_joints()
    calculated_pos = robot.end_effector(fa.get_joints())
    print(calculated_pos)
    print(fa.get_pose())

    q_pre_pick = [-2.98082015e-01, 2.02375423e-01, -2.12616259e-01, -2.20721670e+00, 5.67826931e-04, 2.22226007e+00, 1.87220393e+00]
    fa.goto_joints(q_pre_pick, use_impedance=False, dynamic=False)
    calculated_pos = robot.end_effector(fa.get_joints())
    print(calculated_pos)
    print(fa.get_pose())
    

if __name__ == "__main__":
    main()

