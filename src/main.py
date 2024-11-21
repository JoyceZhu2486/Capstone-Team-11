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

    arm = FrankaArm()

    # calibrate
    calibrator = WorkspaceCalibrator()
    arm.open_gripper()

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
    pen_joints = arm.get_joints()
    arm.close_gripper()
    
    whiteboard_pose = calibrator.calibrate_whiteboard()
    whiteboard_joints = arm.get_joints()

    drop_pose = calibrator.calibrate_drop_location()
    drop_joints = arm.get_joints()
    arm.open_gripper()



    # initialize
    print("Moving to home position...")
    arm.reset_joints()
    arm.open_gripper()
    robot = Robot(arm)
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower()
    
    
    # go to the position to pickup pen:
    print("Moving to pen pickup position")
    pickup_duration = 5
    print(arm.get_pose())
    pickup_joint_waypoints = tg.generate_joint_waypoints(arm.get_joints(),
                                            pen_joints, pickup_duration)
    tf.follow_joint_trajectory(pickup_joint_waypoints)
    time.sleep(0.5)

    # close gripper to pickup pen:
    arm.close_gripper()
    time.sleep(0.5)


    # move pen to the random position on white board
   

    to_board_duration = 5
    to_board_joint_waypoints = tg.generate_joint_waypoints(arm.get_joints(),
                                        whiteboard_joints, to_board_duration)# weird, it seems like it can't move tothe designated position
    tf.follow_joint_trajectory(to_board_joint_waypoints)
    time.sleep(0.5)

else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments

