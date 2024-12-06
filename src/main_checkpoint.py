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

# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

# Get the args container with default values
if __name__ == '__main__':
    args = parser.parse_args()  # get arguments from command line
    
    # for checkpoint

    fa = FrankaArm()

    # calibrate
    calibrator = WorkspaceCalibrator()
    fa.open_gripper()

    # end-effector frame at pen position relative to the base frame
    pen_pos = np.load("pen_holder_pose.npy", allow_pickle=True)
    print(pen_pos)

    # end-effector frame at center of whiteboard relative to the base frame
    center_pos = np.load("whiteboard_pose.npy", allow_pickle=True)
    print(center_pos)

    # end-effector frame at drop bin relative to the base frame
    drop_pos = np.load("drop_bin_pose.npy", allow_pickle=True)
    print(drop_pos)

    # Perform calibration
    pen_positions = calibrator.calibrate_pen_holders()
    pen_joints = fa.get_joints()
    fa.close_gripper()
    
    whiteboard_pose = calibrator.calibrate_whiteboard()
    whiteboard_joints = fa.get_joints()

    drop_pose = calibrator.calibrate_drop_location()
    drop_joints = fa.get_joints()
    fa.open_gripper()
    print(whiteboard_joints)



    # initialize
    print("Moving to home position...")
    fa.reset_joints()
    fa.open_gripper()
    robot = Robot()
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower()
    
    
    # go to the position to pickup pen:
    print("Moving to pen pickup position")
    pickup_duration = 5
    print(fa.get_pose())
    # pickup_joint_waypoints = tg.generate_joint_waypoints(fa.get_joints(),
    #                                         pen_joints, pickup_duration)
    # tf.follow_joint_trajectory(pickup_joint_waypoints)
    pickup_trapezoidal = tg.generate_trapezoidal_trajectory(fa.get_joints(), pen_joints, pickup_duration)
    tf.follow_joint_trajectory(pickup_trapezoidal)
    time.sleep(0.5)
    

    # close gripper to pickup pen:
    fa.close_gripper()
    time.sleep(0.5)


    # move pen to the random position on white board
   

    to_board_duration = 5
    # to_board_joint_waypoints = tg.generate_joint_waypoints(fa.get_joints(),
    #                                     whiteboard_joints, to_board_duration)# weird, it seems like it can't move tothe designated position
    # tf.follow_joint_trajectory(to_board_joint_waypoints)
    to_board_trapezoidal = tg.generate_trapezoidal_trajectory(fa.get_joints(), whiteboard_joints, to_board_duration)
    tf.follow_joint_trajectory(to_board_trapezoidal)
    print(fa.get_joints)
    time.sleep(0.5)

else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments