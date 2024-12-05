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
    rmatrix[0][3] = pose[0]
    rmatrix[1][3] = pose[1]
    rmatrix[2][3] = pose[2]
    print(rmatrix)
    return rmatrix

def matrix_to_translation(matrix):
    return np.array([matrix[0][3],matrix[1][3], matrix[2][3]])

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
    robot = Robot()
    input("Press Enter to open gripper and reset joints")
    arm.open_gripper()
    arm.reset_joints()

    # calibrate
    # calibrator = WorkspaceCalibrator() 
    # bin_pose = calibrator.calibrate_pen_holders()
    # arm.close_gripper()
    # whiteboard_pose = calibrator.calibrate_whiteboard()
    # drop_pose = calibrator.calibrate_drop_location()
    # arm.open_gripper()

    bin_pose = np.load("pen_holder_pose.npy", allow_pickle=True)
    whiteboard_pose = np.load("whiteboard_pose.npy", allow_pickle=True)
    drop_pose = np.load("drop_bin_pose.npy", allow_pickle=True)
    home_matrix = robot.forward_kinematics(RobotConfig.HOME_JOINTS)[:,:,-1]
    pen_matrix = pose_to_matrix(home_matrix, bin_pose)
    # whiteboard_pos_matrix = pos_to_matrix(whiteboard_pose)
    # drop_pos_matrix = pos_to_matrix(drop_pose)

    # move back to home position
    print("Moving back to home position...")
    arm.reset_joints()
    arm.open_gripper()
    cur_joints = RobotConfig.HOME_JOINTS
    cur_position = robot.forward_kinematics(cur_joints)[:,:,-1]

    # go to the position to pickup pen:
    print("Moving to pen pickup position")
    pickup_duration = 3
    pen_joints = robot._inverse_kinematics(pen_matrix, cur_position)
    pickup_joint_waypoints = tg.generate_joint_waypoints(cur_position,
                                            pen_joints, pickup_duration)
    tf.follow_joint_trajectory(pickup_joint_waypoints)
    cur_joints = pen_joints
    cur_position = robot.forward_kinematics(cur_joints)[:,:,-1]

    # close gripper to pickup pen:
    print("Closing gripper")
    arm.close_gripper()

    # move pen above the pen holder
    print("Moving pen vertically")
    lift_duration = 3
    pen_above_matrix = pen_matrix
    pen_above_matrix[2][3] = pen_above_matrix[2][3] + 10
    pen_above_joints = robot._inverse_kinematics(pen_above_matrix, cur_position)
    pen_above_waypoints = tg.generate_cartesian_waypoints(matrix_to_translation(pen_above_matrix), matrix_to_translation(pen_matrix), cur_position, lift_duration)
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