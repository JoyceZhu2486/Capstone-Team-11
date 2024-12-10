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
    
    HOMEJOINTS = np.array([-3.91461865e-04, -7.85867822e-01, 1.17114550e-04, -2.35666438e+00,
        -6.39495011e-04,  1.57148656e+00,  7.85797890e-01])

    # initialize
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower()
    fa = FrankaArm()
    robot = Robot()
    input("Press Enter to 'open gripper' and reset joints")
    fa.open_gripper()
    fa.reset_joints()

    # calibrate
    # calibrator = WorkspaceCalibrator() 
    # calibrator.calibrate_pen_holders()
    # fa.goto_joints(HOMEJOINTS)
    # input("Press enter to close gripper")
    # fa.close_gripper()
    # print(calibrator.calibrate_whiteboard())
    # calibrator.calibrate_drop_location()
    # fa.open_gripper()

    bin_pose = np.array([0,0,0])
    whiteboard_pose = np.eye(4)
    drop_pose = np.array([0,0,0])

    # data loaded from npy files
    pickup_pose = np.load("pen_holder_pose.npy", allow_pickle=True)
    whiteboard_pose = np.load("whiteboard_pose.npy", allow_pickle=True)
    print(whiteboard_pose)
    drop_pose = np.load("drop_bin_pose.npy", allow_pickle=True)

    # move back to home position and reset
    print("Moving back to home position...")
    fa.reset_joints()
    fa.open_gripper()

    # current home joints for calculation
    home_joints = np.copy(HOMEJOINTS)
    # print("The current joint is:", home_joints)
    # the current pose calculated from fk
    home_position = robot.forward_kinematics(home_joints)[:,:,-1]
    # print("The current pose is:",home_position)

    all_pens_done = False
    pen_number = 0
    while(not all_pens_done):
        # make a copy of the home joints
        home_joints_changed = np.copy(home_joints)

        # go to the position to pickup pen:
        pickup_pose_changed = np.copy(pickup_pose)
        pickup_pose_changed[0] += 0.027 * pen_number
        pickup_pose_higher_changed = np.copy(pickup_pose_changed)
        pickup_pose_higher_changed[2] += 0.17
        # if(pen_number==3):
        #     pickup_pose_changed[0] += 0.002
        # print("the z-value of pick_up pose is:", pickup_pose_changed[2])
        pen_higher_matrix = pose_to_matrix(home_position, pickup_pose_higher_changed) # add pickup_pose xyz value to obtain the target pickup pose

        pickup_duration = 4.5
        pick_higher_joints = robot._inverse_kinematics(pen_higher_matrix, home_joints_changed) # joints at target pickup pose
        # generate and follow pre pick trajectory
        pre_pickup_trap_waypoints = tg.generate_trapezoidal_trajectory(home_joints, pick_higher_joints, pickup_duration)
        input("Press enter to start moving to pre pick position:")
        tf.follow_joint_trajectory(pre_pickup_trap_waypoints)
        time.sleep(0.2)

        # move end effector down
        pickup_cartesian_waypoints = tg.generate_cartesian_waypoints(pickup_pose_higher_changed, pickup_pose_changed, 3, pick_higher_joints)
        input("Press enter to move down to pick position:")
        tf.follow_joint_trajectory(pickup_cartesian_waypoints)
        time.sleep(0.2)

        # close gripper to pickup pen:
        input("Press Enter to close the gripper")
        print("Closing gripper")
        fa.close_gripper()

        # move pen above the pen holder
        above_pickup_waypoints = tg.generate_cartesian_waypoints(pickup_pose_changed, pickup_pose_higher_changed, 3, pickup_cartesian_waypoints[-1])

        input("Press enter to go to above pickup position")
        print("Moving pen vertically")
        tf.follow_joint_trajectory(above_pickup_waypoints)
        time.sleep(0.2)

        # trajectory from the point above pen_pickup to home joints
        above_pickup_to_home = tg.generate_trapezoidal_trajectory(above_pickup_waypoints[-1], home_joints, 4.5)
        tf.follow_joint_trajectory(above_pickup_to_home)
        time.sleep(0.2)

        # move from home position to whiteboard origin
        print("Moving to whiteboard orgin...")
        whiteboard_offset_y = -pen_number*0.02
        to_board_duration = 5
        whiteboard_matrix = np.copy(whiteboard_pose)
        whiteboard_xyz_offested = tg.whiteboard_to_world(np.array([0,whiteboard_offset_y,0]), np.copy(whiteboard_matrix))
        whiteboard_matrix_offested = pose_to_matrix(np.copy(whiteboard_matrix), whiteboard_xyz_offested)
        print("whiteboard_matrix_offested \n", whiteboard_matrix_offested)
        home_joints_changed = np.copy(home_joints)
        whiteboard_joints = robot._inverse_kinematics(np.copy(whiteboard_matrix_offested), home_joints_changed)
        print("whiteboard_matrix_offested \n", whiteboard_matrix_offested)
        print("home pose:\n", fa.get_pose())
        to_board_trap_waypoints = tg.generate_trapezoidal_trajectory(home_joints,
                                            whiteboard_joints, to_board_duration)

        # print("the whiteboard joint is:", whiteboard_joints)
        input("Press enter to go to whiteboard origin")
        tf.follow_joint_trajectory(to_board_trap_waypoints)
        time.sleep(0.2)
        print("whiteboard_matrix_offested \n", whiteboard_matrix_offested)
        print("whiteboard pose:\n", fa.get_pose())

        
        # # draw line 
        straight_line_traj = tg.generate_straight_line(np.array([0,whiteboard_offset_y,0]), np.array([0.1,whiteboard_offset_y,0]), whiteboard_joints, 3.5, whiteboard_pose)
        tf.follow_joint_trajectory(straight_line_traj)
        time.sleep(0.2)

        # lift the pen
        lift_pen_traj = tg.generate_straight_line(np.array([0.1,whiteboard_offset_y,0]), np.array([0.1,whiteboard_offset_y,-0.01]), straight_line_traj[-1], 1, whiteboard_pose)
        input("Press enter to lift the pen and move to new drawing point")
        tf.follow_joint_trajectory(lift_pen_traj)
        time.sleep(0.2)
        
        # move the pen to a new point
        move_pen_traj = tg.generate_straight_line(np.array([0.1,whiteboard_offset_y,-0.01]), np.array([0.1,-0.05+whiteboard_offset_y,-0.01]), lift_pen_traj[-1], 1, whiteboard_pose)
        tf.follow_joint_trajectory(move_pen_traj)
        time.sleep(0.2)

        # put the pen onto board
        putdown_pen_traj = tg.generate_straight_line(np.array([0.1,-0.05+whiteboard_offset_y,-0.01]), np.array([0.1,-0.05+whiteboard_offset_y,0.0]), move_pen_traj[-1], 2, whiteboard_pose)
        tf.follow_joint_trajectory(putdown_pen_traj)
        time.sleep(0.2)

        # draw curve
        curve_traj = tg.generate_curve(np.array([0.1,-0.05+whiteboard_offset_y,0.0]), np.array([0,-0.05+whiteboard_offset_y,0]), 5, whiteboard_pose, putdown_pen_traj[-1])
        tf.follow_joint_trajectory(curve_traj)
        time.sleep(0.2)


        # trajectory from the point on whiteboard to home joints
        whiteboard_to_home = tg.generate_trapezoidal_trajectory(curve_traj[-1], home_joints, 4.5)
        tf.follow_joint_trajectory(whiteboard_to_home)
        time.sleep(0.2)
        if(pen_number != 0):
            # move to drop position
            print("Moving to drop position...")
            drop_matrix = pose_to_matrix(np.copy(home_position), drop_pose)
            print("drop_matrix before rotation:\n", drop_matrix)

            # Define the rotation matrix around y-axis
            drop_theta = -np.pi/3.5
            rotation_matrix = np.array([
                [ np.cos(drop_theta), 0, np.sin(drop_theta), 0],
                [     0, 1,     0, 0],
                [-np.sin(drop_theta), 0, np.cos(drop_theta), 0],
                [     0, 0,     0, 1]
            ])
            drop_matrix_rotate = rotation_matrix @ drop_matrix
            drop_matrix_rotate = pose_to_matrix(np.copy(drop_matrix_rotate), drop_pose)
            home_joints_changed = np.copy(home_joints)
            drop_joints = robot._inverse_kinematics(np.copy(drop_matrix_rotate), home_joints_changed)
            print("drop_matrix rotated:\n", drop_matrix_rotate)
            print("drop_pose:\n", drop_pose)
            print("drop_joints:\n", drop_joints)
            to_drop_trap_traj = tg.generate_trapezoidal_trajectory(home_joints, drop_joints, 9)
            input("Press enter to go to drop position")
            tf.follow_joint_trajectory(to_drop_trap_traj)
            time.sleep(0.2)
            input("Press enter to open gripper")
            fa.open_gripper()
            
            # return to home position
            drop_to_home = tg.generate_trapezoidal_trajectory(drop_joints, home_joints, 6)
            tf.follow_joint_trajectory(drop_to_home)
            time.sleep(0.2)
            print("SUCCESS!!!")

        else:
            # move to before drop pen holder position
            print("Moving to pen holder position...")
            holder_drop_higher = np.copy(pickup_pose)
            holder_drop_higher[2] += 0.17
            holder_drop_higher_matrix = pose_to_matrix(np.copy(home_position), holder_drop_higher)
            home_joints_changed = np.copy(home_joints)
            holder_drop_higher_joints = robot._inverse_kinematics(np.copy(holder_drop_higher_matrix), home_joints_changed)
            print("before_drop_joints:\n", holder_drop_higher_joints)
            holder_drop_higher_trap_traj = tg.generate_trapezoidal_trajectory(home_joints, holder_drop_higher_joints, 5)
            input("Press enter to go to before holder drop position")
            tf.follow_joint_trajectory(holder_drop_higher_trap_traj)
            time.sleep(0.2)

            # move to drop pen holder position
            holder_drop = np.copy(pickup_pose)
            holder_drop[2] += 0.05
            holder_drop_waypoints= tg.generate_cartesian_waypoints(holder_drop_higher, holder_drop, 2.5, holder_drop_higher_trap_traj[-1])
            input("Press enter to go to holder drop position")
            tf.follow_joint_trajectory(holder_drop_waypoints)

            input("Press enter to open gripper")
            fa.open_gripper()
            
            # return to home position
            drop_to_home = tg.generate_trapezoidal_trajectory(holder_drop_waypoints[-1], home_joints, 6)
            tf.follow_joint_trajectory(drop_to_home)
            print("SUCCESS!!!")

        pen_number += 1
        if(pen_number == 4):
            all_pens_done = True

    print("ALL PENS DONE")

else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments
