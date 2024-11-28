import argparse
import time
from robot  import *
from calibrate_workspace import *
from motion_planner import *
from utils  import *

def main():
    fa = FrankaArm()
    robot = Robot()
    time.sleep(0.5)

    fa.reset_joints()
    real_pose = fa.get_pose()
    real_joints = fa.get_joints()
    calculated_pos = robot.end_effector(real_joints)
    calculated_joints = robot._inverse_kinematics(real_pose)
    print("real pose:           ", real_pose)
    print("fk calculated pose:  ", calculated_pos)
    print()
    print("real joints:         ", real_joints)
    print("ik calculated joints:", calculated_joints)
    print()
    

    q_pre_pick = [-2.98082015e-01, 2.02375423e-01, -2.12616259e-01, -2.20721670e+00, 5.67826931e-04, 2.22226007e+00, 1.87220393e+00]
    fa.goto_joints(q_pre_pick, use_impedance=False, dynamic=False)
    real_pose = fa.get_pose()
    real_joints = fa.get_joints()
    calculated_pos = robot.end_effector(real_joints)
    calculated_joints = robot._inverse_kinematics(real_pose)
    print("real pose:           ", real_pose)
    print("fk calculated pose:  ", calculated_pos)
    print()
    print("real joints:         ", real_joints)
    print("ik calculated joints:", calculated_joints)
    print()
    

if __name__ == "__main__":
    main()