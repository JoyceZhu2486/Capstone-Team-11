"""
    You will use this script for the final demo.
    Implement your code flow here.
    Your main fucntion can take in different command line arguments to run different parts of your code.
"""
import argparse
import time
from robot  import *
from calibrate_workspace import *
from motion_planner import *
from utils  import *
import numpy as np
import matplotlib.pyplot as plt


def pose_to_matrix(rmatrix, pose):
    """
    Updates the translation part of a 4x4 transformation matrix.

    Parameters
    ----------
    rmatrix : np.ndarray
        A 4x4 transformation matrix.
    pose : np.ndarray
        A 3-element array or list representing [x, y, z].

    Returns
    -------
    np.ndarray
        A new 4x4 transformation matrix with updated translation components.
    """
    # Validate input matrix
    if rmatrix.shape != (4, 4):
        raise ValueError("Input rmatrix must be a 4x4 matrix.")
    
    # Validate pose length
    if len(pose) != 3:
        raise ValueError("Input pose must be a 3-element array or list [x, y, z].")

    # Copy and update translation components
    result = np.copy(rmatrix)
    result[:3, 3] = pose  # Update the translation part directly
    return result


def matrix_to_xyz(matrix):
    """
    Extracts the translation (x, y, z) from a 4x4 transformation matrix.

    Parameters
    ----------
    matrix : np.ndarray
        A 4x4 transformation matrix.

    Returns
    -------
    np.ndarray
        A 3-element array containing the translation components [x, y, z].
    """
    # Validate input matrix
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 transformation matrix.")

    # Extract and return translation components
    return matrix[:3, 3]


# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

# Get the args container with default values
if __name__ == '__main__':
    args = parser.parse_args()  # get arguments from command line
    
    # initialize
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower()
    fa = FrankaArm()
    robot = Robot()
    input("Press Enter to open gripper and reset joints")
    fa.open_gripper()
    fa.reset_joints()

    # # calibrate
    # calibrator = WorkspaceCalibrator() 
    # bin_pose = calibrator.calibrate_pen_holders()
    # actual_penjoints = fa.get_joints()
    # fa.close_gripper()
    # whiteboard_pose = calibrator.calibrate_whiteboard()
    # drop_pose = calibrator.calibrate_drop_location()
    # fa.open_gripper()

    # data loaded from npy files
    pickup_pose = np.load("pen_holder_pose.npy", allow_pickle=True)
    whiteboard_pose = np.load("whiteboard_pose.npy", allow_pickle=True)
    drop_pose = np.load("drop_bin_pose.npy", allow_pickle=True)

    # move back to home position and reset
    print("Moving back to home position...")
    fa.reset_joints()
    fa.open_gripper()

    # current home joints for calculation
    home_joints = fa.get_joints()
    # make a copy of the home joints
    home_joints_changed = np.copy(home_joints)
    print("The current joint is:", home_joints)

    # the current pose calculated from fk
    cur_position = robot.forward_kinematics(home_joints)[:,:,-1]
    print("The current pose is:",cur_position)

    # go to the position to pickup pen:
    input("Calibration pprocess finished. Press enter to start moving:")
    print("Moving to pen pickup position")
    pen_matrix = pose_to_matrix(cur_position, pickup_pose) # add pickup_pose xyz value to obtain the target pickup pose
    pickup_duration = 4
    pen_joints = robot._inverse_kinematics(pen_matrix, home_joints_changed) # joints at target pickup pose
    print("Pen joints when picking pen:", pen_joints)
    # print("The current joint is:", cur_joints)
    # print("The pen pick joint is:\n",pen_joints)

    # generate and follow trajectory
    pickup_trap_waypoints = tg.generate_trapezoidal_trajectory(home_joints, pen_joints, pickup_duration)
    tf.follow_joint_trajectory(pickup_trap_waypoints)
    input("Press Enter to close the gripper")

    # close gripper to pickup pen:
    print("Closing gripper")
    fa.close_gripper()

    # move pen above the pen holder by 1
    print("Moving pen vertically")
    lift_duration = 2
    above_pickup_matrix = np.copy(pen_matrix)
    above_pickup_matrix[2][3] += 0.13 # add 13cm to the z-axis
    pen_joints_changed = np.copy(pen_joints) # copy of the pickup joint

    # use ik to calculate the joints above the pickup point
    above_pickup_joints = robot._inverse_kinematics(above_pickup_matrix, pen_joints_changed)

    # calculate the xyz values for cartesian trajectory
    pen_xyz = matrix_to_xyz(pen_matrix)
    above_pickup_xyz = matrix_to_xyz(above_pickup_matrix)
    above_pickup_waypoints = tg.generate_cartesian_waypoints(pen_xyz, above_pickup_xyz, lift_duration, pen_joints)

    input("Press enter to go to above pickup position")
    tf.follow_joint_trajectory(above_pickup_waypoints)

    # trajectory from the point above pen_pickup to home joints
    above_pickup_to_home = tg.generate_trapezoidal_trajectory(above_pickup_joints, home_joints, 8)
    input("Press enter to return home position")
    tf.follow_joint_trajectory(above_pickup_to_home)


    # move from home position to whiteboard origin
    print("Moving to whiteboard orgin...")
    to_board_duration = 6
    whiteboard_matrix = np.copy(whiteboard_pose)
    home_joints_changed = np.copy(home_joints)
    whiteboard_joints = robot._inverse_kinematics(whiteboard_matrix, home_joints_changed)
    to_board_trap_waypoints = tg.generate_trapezoidal_trajectory(home_joints,
                                        whiteboard_joints, to_board_duration)

    print("the whiteboard joint is:", whiteboard_joints)
    input("Press enter to go to whiteboard origin")
    tf.follow_joint_trajectory(to_board_trap_waypoints)


    # TODO: draw line 
    straight_line_traj = tg.generate_straight_line(whiteboard_pose, whiteboard_joints , 5, whiteboard_pose)
    input("Press enter to draw straight line")
    tf.follow_joint_trajectory(straight_line_traj)

    # TODO: draw curve
    # cur_joints = above_pickup_joints



    # from home position
    # move to drop position
    # print("Moving to drop position...")
    # to_drop_duration = 10
    # drop_matrix = pose_to_matrix(cur_position, drop_pose)
    # drop_joints = robot._inverse_kinematics(drop_matrix, cur_joints)
    # to_drop_joint_waypoints = tg.generate_joint_waypoints(cur_joints,
    #                                         drop_joints, to_drop_duration)
    # tf.follow_joint_trajectory(to_drop_joint_waypoints) 

else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments