import argparse
import time
from robot  import *
from calibrate_workspace import *
from motion_planner import *
from utils import _rotation_to_quaternion
import numpy as np
from utils import *

def main():

    fa = FrankaArm()
    robot = Robot()
    time.sleep(0.5)

    fa.reset_joints()

    # q_pre_pick = [-2.98082015e-01, 2.02375423e-01, -2.12616259e-01, -2.20721670e+00, 5.67826931e-04, 2.22226007e+00, 1.87220393e+00]
    # fa.goto_joints(q_pre_pick, use_impedance=False, dynamic=False)
    real_pose = fa.get_pose()
    real_joints = fa.get_joints()

    print("real pose:\n", real_pose)
    print("real pose translation :\n", real_pose.translation)
    print("real pose rotation:\n", real_pose.rotation)

    frames = robot.forward_kinematics(real_joints)
    fk_ee = frames[...,-1]

    calculated_joints = robot._inverse_kinematics(real_pose, real_joints,True)
    print("calculated_joints",calculated_joints)
    #fa.goto_joints(calculated_joints, use_impedance=False, dynamic=False)
    print("real_joints",real_joints)
    print("-----------------------------------")
    print("frame with flange and offset: \n", fk_ee)

    calculated_frames = robot.forward_kinematics(calculated_joints)
    fk_ee_calculated = frames[...,-1]
    print("-----------------------------------")
    print("frame with flange and offset: \n", fk_ee_calculated)



    # fk_pprevious_pose = frames[...,-4]
    # fk_previous_pose = frames[...,-3]
    # fk_pose_without_flange = frames[...,-2]

    # print("real pose:\n", real_pose)
    # print("real joints:\n", real_joints)
    # print("-----------------------------------")
    # print("ik calculated joints:\n", calculated_joints)
    # print()
    # print("1 frame: \n", frames[...,-9])
    # print("-----------------------------------")
    # print("2 frame: \n", frames[...,-8])
    # print("-----------------------------------")
    # print("3 frame: \n", frames[...,-7])
    # print("-----------------------------------")
    # print("4 frame: \n", frames[...,-6])
    # print("-----------------------------------")
    # print("5 frame: \n", frames[...,-5])
    # print("-----------------------------------")
    # print("6 frame: \n", fk_pprevious_pose)
    # print("-----------------------------------")
    # print("last joint without flange\n", fk_previous_pose)
    # print("-----------------------------------")
    # print("frame with flange: \n", fk_pose_with_flange)
    # print("-----------------------------------")
    # print("frame with flange and offset: \n", fk_pose_with_flange)

    # frames = robot.forward_kinematics(calculated_joints)
    # fk_pose_without_flange = frames[...,-2]
    # fk_pose_with_flange = frames[...,-1]
    
    # print("-----------------------------------")
    # print("frame with flange and offset: \n", fk_pose_with_flange)

    # qua = _rotation_to_quaternion(fk_pose_without_flange[:3,:3])
    # # print("quaternion:\n", qua)
    # qua_flange = _rotation_to_quaternion(fk_pose_with_flange[:3,:3])
    # # print("quaternion with flange:\n", qua_flange)
    # print("x,y,z,roll,pitch,yaw", robot.end_effector(dh_parameters, real_joints))
    # print("frames:\n", frames)
    # print()
    

    

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