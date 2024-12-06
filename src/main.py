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


def matrix_to_translation(matrix):
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

    # calibrate
    # calibrator = WorkspaceCalibrator() 
    # bin_pose = calibrator.calibrate_pen_holders()
    # actual_penjoints = fa.get_joints()
    # fa.close_gripper()
    # whiteboard_pose = calibrator.calibrate_whiteboard()
    # drop_pose = calibrator.calibrate_drop_location()
    # fa.open_gripper()

    bin_pose = np.load("pen_holder_pose.npy", allow_pickle=True)
    whiteboard_pose = np.load("whiteboard_pose.npy", allow_pickle=True)
    drop_pose = np.load("drop_bin_pose.npy", allow_pickle=True)


    # move back to home position
    print("Moving back to home position...")
    fa.reset_joints()
    fa.open_gripper()
    # current home joints
    cur_joints = fa.get_joints()
    print(cur_joints)

    # the current pose
    cur_position = robot.forward_kinematics(cur_joints)[:,:,-1]
    print(cur_position)

    # go to the position to pickup pen:
    print("Moving to pen pickup position")

    pen_matrix = pose_to_matrix(cur_position, bin_pose)
    pickup_duration = 30
    pen_joints = robot._inverse_kinematics(pen_matrix, cur_joints)
    print("penjoints:\n",pen_joints)
    # print("actual penjoints:\n",actual_penjoints)
    input()
    fa.goto_joints(pen_joints)
    input()

        
    pickup_joint_waypoints = tg.generate_trapezoidal_trajectory(cur_joints, pen_joints, pickup_duration)
    print(pickup_joint_waypoints[0])
    print(pickup_joint_waypoints[-1])
    
    tf.follow_joint_trajectory(pickup_joint_waypoints)
    cur_joints = pen_joints
    cur_position = robot.forward_kinematics(cur_joints)[:,:,-1]

    # close gripper to pickup pen:
    print("Closing gripper")
    fa.close_gripper()

    # move pen above the pen holder
    print("Moving pen vertically")
    lift_duration = 3
    pen_above_matrix = pen_matrix
    pen_above_matrix[2][3] = pen_above_matrix[2][3] + 10
    pen_above_joints = robot._inverse_kinematics(pen_above_matrix, cur_position)
    pen_above_waypoints = tg.generate_trapezoidal_trajectory(cur_joints, pen_joints, pickup_duration)
    tf.follow_joint_trajectory(pen_above_waypoints)
    cur_joints = pen_above_joints
    cur_position = robot.forward_kinematics(cur_joints)[:,:,-1]

    # move from the above pen holder position to home position of whiteboard
    print("Moving to whiteboard")
    to_board_duration = 3
    whiteboard_matrix = whiteboard_pose
    whiteboard_joints = robot._inverse_kinematics(whiteboard_matrix, cur_joints)
    to_board_joint_waypoints = tg.generate_joint_waypoints(pen_above_joints,
                                        whiteboard_joints, to_board_duration)
    tf.follow_joint_trajectory(to_board_joint_waypoints)
    cur_joints = whiteboard_joints
    cur_position = robot.forward_kinematics(cur_joints)[:,:,-1]


    # TODO: draw line 
    # TODO: draw curve
    cur_joints = pen_above_joints

    # move to drop position
    print("Moving to drop position")
    to_drop_duration = 3
    drop_matrix = pose_to_matrix(cur_position, drop_pose)
    drop_joints = robot._inverse_kinematics(drop_matrix, cur_joints)
    to_drop_joint_waypoints = tg.generate_joint_waypoints(cur_joints,
                                            drop_joints, to_drop_duration)
    tf.follow_joint_trajectory(to_drop_joint_waypoints) 

else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments

main.py