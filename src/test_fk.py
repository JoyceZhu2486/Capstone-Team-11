import argparse
import time
from robot  import *
from calibrate_workspace import *
from motion_planner import *
from utils  import *
import numpy as np
import math

def main():
    
    dh_parameters = np.array(  [[0,0,0.333,0],
                                [0,-math.pi/2,0,0],
                                [0,math.pi/2,0.316,0],
                                [0.0825,math.pi/2,0,0],
                                [-0.0825,-math.pi/2,0.384,0],
                                [0,math.pi/2,0,0],
                                [0.088,math.pi/2,0,0],
                                [0,0,0.107,0]])


    fa = FrankaArm()
    robot = Robot(fa)
    time.sleep(0.5)

    fa.reset_joints()
    real_pose = fa.get_pose()
    real_joints = fa.get_joints()
    calculated_pos = robot.forward_kinematcis(dh_parameters, real_joints)[...,-1]
    # calculated_joints = robot._inverse_kinematics(real_pose, real_joints)
    print("real pose:           \n", real_pose)
    print("fk calculated pose:  \n", calculated_pos)
    # print()
    # print("real joints:         ", real_joints)
    # print("ik calculated joints:", calculated_joints)
    print()
    

    # q_pre_pick = [-2.98082015e-01, 2.02375423e-01, -2.12616259e-01, -2.20721670e+00, 5.67826931e-04, 2.22226007e+00, 1.87220393e+00]
    # fa.goto_joints(q_pre_pick, use_impedance=False, dynamic=False)
    # real_pose = fa.get_pose()
    # real_joints = fa.get_joints()
    # calculated_pos = robot.end_effector(dh_parameters, real_joints)
    # # calculated_joints = robot._inverse_kinematics(real_pose, real_joints)
    # print("real pose:           ", real_pose)
    # print("fk calculated pose:  ", calculated_pos)
    # print()
    # print("real joints:         ", real_joints)
    # # print("ik calculated joints:", calculated_joints)
    # print()
    

if __name__ == "__main__":
    main()