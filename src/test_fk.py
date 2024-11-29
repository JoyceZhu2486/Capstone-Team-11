import argparse
import time
from robot  import *
from calibrate_workspace import *
from motion_planner import *
from utils import _rotation_to_quaternion
import numpy as np
import math
from utils import *

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
    # q_pre_pick = [-2.98082015e-01, 2.02375423e-01, -2.12616259e-01, -2.20721670e+00, 5.67826931e-04, 2.22226007e+00, 1.87220393e+00]
    # fa.goto_joints(q_pre_pick, use_impedance=False, dynamic=False)
    real_pose = fa.get_pose()
    real_joints = fa.get_joints()

    frames,fk_pose_with_flange = robot.forward_kinematcis(dh_parameters, real_joints)

    # calculated_pos = robot.forward_kinematcis(dh_parameters, real_joints)[...,-1]

    flange = np.array([[1,0,0,0],
                       [0,1,0,0],
                       [0,0,1,0.107],
                       [0,0,0,1]])
    fk_pose_without_flange = frames[...,-1]
    # calculated_joints = robot._inverse_kinematics(real_pose, real_joints)
    print("real pose:           \n", real_pose)
    print("-----------------------------------")
    print("fk calculated pose: \n", fk_pose_without_flange)
    print("fk calculated with flange: \n", fk_pose_with_flange)
    # qua = _rotation_to_quaternion(fk_pose_without_flange[:3,:3])
    # # print("quaternion:\n", qua)
    # qua_flange = _rotation_to_quaternion(fk_pose_with_flange[:3,:3])
    # # print("quaternion with flange:\n", qua_flange)
    # print("x,y,z,roll,pitch,yaw", robot.end_effector(dh_parameters, real_joints))
    # print("frames:\n", frames)
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