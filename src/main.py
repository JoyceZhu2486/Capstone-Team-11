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


def pos_to_matrix():
    pass


# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

# Get the args container with default values
if __name__ == '__main__':
    args = parser.parse_args()  # get arguments from command line
    
    # initialize
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower()
    arm = FrankaArm()
    robot = Robot(arm)
    input("Press Enter to open gripper and reset joints")
    arm.open_gripper()
    arm.reset_joints()

    # calibrate
    calibrator = WorkspaceCalibrator() 
    pen_pos = calibrator.calibrate_pen_holders()
    # pen_joints = arm.get_joints()
    arm.close_gripper()
    whiteboard_pos = calibrator.calibrate_whiteboard()
    # whiteboard_joints = arm.get_joints()
    drop_pos = calibrator.calibrate_drop_location()
    arm.open_gripper()
    # drop_joints = arm.get_joints()
    # print(whiteboard_joints)

    pen_pos = np.load("pen_holder_pose.npy", allow_pickle=True)
    whiteboard_pos = np.load("whiteboard_pose.npy", allow_pickle=True)
    drop_pos = np.load("drop_bin_pose.npy", allow_pickle=True)
    pen_pos_matrix = pos_to_matrix(pen_pos)
    whiteboard_pos_matrix = pos_to_matrix(whiteboard_pos)
    drop_pos_matrix = pos_to_matrix(drop_pos)

    # move back to home position
    print("Moving back to home position...")
    arm.reset_joints()
    arm.open_gripper()
    home_position = arm.get_joints()

    # go to the position to pickup pen:
    print("Moving to pen pickup position")
    pickup_duration = 5
    pen_joints = robot._inverse_kinematics(pen_pos_matrix, home_position)
    pickup_joint_waypoints = tg.generate_joint_waypoints(home_position,
                                            pen_joints, pickup_duration)
    tf.follow_joint_trajectory(pickup_joint_waypoints)

    # close gripper to pickup pen:
    print("Closing gripper")
    arm.close_gripper()

    # move pen above the pen holder
    print("Moving pen vertically")
    pen_above_matrix = pen_pos_matrix
    above_offset = 10
    pen_above_matrix[2][3] = pen_above_matrix[2][3] + above_offset
    pen_above_joints = robot._inverse_kinematics(pen_above_matrix, home_position)

    # move from the above pen holder position to home position of whiteboard
    print("Moving to whiteboard")
    to_board_duration = 5
    whiteboard_joints = robot._inverse_kinematics(whiteboard_pos_matrix, pen_above_joints)
    to_board_joint_waypoints = tg.generate_joint_waypoints(pen_above_joints,
                                        whiteboard_joints, to_board_duration)
    tf.follow_joint_trajectory(to_board_joint_waypoints)

    # TODO: draw line 
    # TODO: draw curve
    cur_joints = pen_above_joints

    # move to drop position
    print("Moving to drop position")
    to_drop_duration = 5
    drop_joints = robot._inverse_kinematics(drop_pos_matrix, cur_joints)
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