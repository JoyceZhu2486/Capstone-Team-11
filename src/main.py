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
    
    # initialize
    print("Moving to home position...")
    arm = FrankaArm()
    arm.fa.reset_joints()
    robot = Robot(arm)
    cal = WorkspaceCalibrator()
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower()
    
    
    # go to the position to pickup pen:
    print("Moving to pen pickup position")
    pickup_duration = 5
    pickup_joint_waypoints = tg.generate_joint_waypoints(arm.fa.get_pose(),
                                            pen_positions, pickup_duration)
    tf.follow_joint_trajectory(pickup_joint_waypoints)
    time.sleep(0.5)

    # close gripper to pickup pen:
    arm.fa.close_gripper()
    time.sleep(0.5)

    # move pen to the random position pn white board
    to_board_duration = 15
    to_board_joint_waypoints = tg.generate_joint_waypoints(arm.fa.get_pose(),
                                        whiteboard_pose, to_board_duration)
    tf.follow_joint_trajectory(to_board_joint_waypoints)
    time.sleep(0.5)


else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments