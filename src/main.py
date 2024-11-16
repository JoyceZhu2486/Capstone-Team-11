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
    
    # Perform calibration
    pen_positions = calibrator.calibrate_pen_holders()
    pen_joints = arm.get_joints()
    arm.close_gripper()

    whiteboard_pose = calibrator.calibrate_whiteboard()
    whiteboard_joints = arm.get_joints()
    drop_pose = calibrator.calibrate_drop_location()
    arm.open_gripper()


    # initialize
    print("Moving to home position...")
    arm.reset_joints()
    arm.open_gripper()
    robot = Robot(arm)
    tg = TrajectoryGenerator()
    tf = TrajectoryFollower(robot)
    
    
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

    # move pen to the random position pn white board
    to_board_duration = 15
    to_board_joint_waypoints = tg.generate_joint_waypoints(arm.get_joints(),
                                        whiteboard_joints, to_board_duration)
    tf.follow_joint_trajectory(to_board_joint_waypoints)
    time.sleep(0.5)


else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments