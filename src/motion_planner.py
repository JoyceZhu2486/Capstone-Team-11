import sys
sys.path.append("../config")

import sys
sys.path.append("../config")

import numpy as np
from autolab_core import RigidTransform
from robot_config import RobotConfig
from frankapy import FrankaArm
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from robot import *
import rospy

class TrajectoryGenerator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.max_vel = RobotConfig.MAX_VELOCITY
        self.max_acc = RobotConfig.MAX_ACCELERATION
    
    def generate_joint_waypoints(self, start_joint, end_joint, duration):
        """
        Generate a pose trajectory as a series of waypoints using linear interpolation.
        
        This method calculates waypoints at regular intervals over the specified duration
        to create a linear motion from the starting pose to the ending pose. Each waypoint
        is a pose object.

        Parameters
        ----------
        start_pose : RigidTransform
            The starting pose of the trajectory, including position and orientation.
        end_pose : RigidTransform
            The ending pose of the trajectory, including position and orientation.
        duration : float
            The total time over which the trajectory should be executed.

        Returns
        -------
        list of Poses
            A list of poses representing the waypoints of the trajectory.
            Each waypoint is spaced at an interval of 20ms, as defined by `self.dt`.

        """
        print("start joint:")
        print(start_joint)
        print("end joint:")
        print(end_joint)
        num_points = int(duration/0.02) + 1
        joint_trajectory = np.linspace(start_joint, end_joint, int(num_points))
        return joint_trajectory

    def generate_cartesian_waypoints(self, start_pose, end_pose, duration, current_joint):
        """
        Time-parameterize Cartesian trajectory with trapezoidal velocity profile.

        Parameters
        ----------
        cartesian_trajectory : array_like
            Array of poses representing path in Cartesian space
        
        Returns
        ------- 
        array_like
            Time-parameterized trajectory with 20ms spacing

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        Key Requirements:  
        - Timing: Waypoints must be spaced exactly 20ms apart for controller
        - Safety: Stay within MAX_VELOCITY and MAX_ACCELERATION limits
        - Smoothness: Use trapezoidal velocity profile for acceleration/deceleration

        Implementation:
        - Calculate duration based on path length and velocity limits
        - Generate trapezoidal velocity profile with acceleration limits 
        - Ensure 20ms spacing between waypoints
        - For rotations: Use SLERP to interpolate between orientations
        """
        # Number of waypoints
        n_points = int(duration / self.dt) + 1
        # Time steps
        times = np.linspace(0, duration, n_points)
        # Initialize list of waypoints
        cartesian_trap = self.generate_trapezoidal_trajectory(start_pose, end_pose, duration)
        # Assume start_pose and end_pose are arrays containing x,y,z
        waypoints = np.zeros(len(cartesian_trap))
        waypoints[0] = current_joint
        current_matrix = robot.forward_kinematics(current_joint)[:,:,-1]
        next_matrix = current_matrix
        next_matrix[2][0] = cartesian_trap[0][0]
        next_matrix[2][1] = cartesian_trap[0][1]
        next_matrix[2][2] = cartesian_trap[0][2]

        for i in range(1, len(cartesian_trap)):
            robot = Robot()
            next_matrix[2][0] = cartesian_trap[i][0]
            next_matrix[2][1] = cartesian_trap[i][1]
            next_matrix[2][2] = cartesian_trap[i][2]
            waypoints[i] = robot._inverse_kinematics(next_matrix, waypoints[i-1])
        return waypoints
    
    def generate_straight_line(self, start_pose, current_joint, duration=None):
        """
        This function creates a smooth straight-line trajectory for the robot's end-effector to follow.

        Parameters
        ----------
        start_pose : np.ndarray
            The starting pose of the line in Cartesian space (4x4 transformation matrix).
        current_joint : np.ndarray
            The current joint configuration of the robot.
        duration : float, optional
            The total duration over which the line should be drawn. If not specified,
            the speed defaults to a pre-defined value.

        Returns
        -------
        np.ndarray
            Joint trajectory for the straight-line path.

        Notes
        -----
        - The method uses both translational and rotational dynamics to ensure the end-effector
        is properly aligned during the motion.
        - The method generates Cartesian waypoints and converts them to joint trajectories using IK.
        """
        # Define offsets for the straight line in the x and y directions
        x_offset = 0.1  # Move 10 cm along the x-axis
        y_offset = 0.05  # Move 5 cm along the y-axis

        # Copy the start pose and calculate the end pose
        end_pose = start_pose.copy()
        end_pose[0, 3] += x_offset
        end_pose[1, 3] += y_offset

        # Calculate the distance between start and end points
        start_point = start_pose[:3, 3]
        end_point = end_pose[:3, 3]
        distance = np.linalg.norm(end_point - start_point)

        # Set default duration if not provided
        if duration is None:
            speed = 0.1  # Default speed (m/s)
            duration = distance / speed

        # Generate Cartesian waypoints and convert to joint trajectory
        joint_trajectory = self.generate_cartesian_waypoints(
            start_pose, end_pose, duration, current_joint
        )

        return joint_trajectory
        

    def generate_curve(self, points, duration=None):
        """
        This function creates a smooth curved trajectory for the robot's end-effector to follow.

        Parameters
        ----------
        points : list of array_like
            A list of points that define the curve. Each point is a position in Cartesian space.
        duration : float, optional
            The total duration over which the curve should be drawn. If not specified,
            the speed should default to a pre-defined value.

        Return
        ------
        array_like
            Input to either generate_cartesian_waypoints or follow_cartesian_trajectory
            
        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Notes
        -----
        - The trajectory should smoothly interpolate between all provided points.
        - Consider using higher-order interpolation techniques or spline curves for smoother results.

        Hints
        -----
        - Implement a spline or Bezier curve interpolation to generate intermediate waypoints.
        - Adjust the robot's end-effector orientation to ensure it is appropriate for the drawing task.
        - Ensure the trajectory is feasible within the robot's motion capabilities, particularly regarding
        joint limits and maximum velocities.
        """
        raise NotImplementedError("Implement curve drawing trajectory")
    
    def generate_trapezoidal_trajectory(self, q_start, q_end, duration):
        """
        Generate trajectory with trapezoidal velocity profile.
        
        From writeup:
        - Must have proper acceleration/deceleration
        - Must maintain smooth motion
        - Must output at 20ms intervals
        
        Parameters
        ----------
        q_start : np.ndarray
            Starting joint configuration
        q_end : np.ndarray
            Ending joint configuration
        max_vel : float
            Maximum allowed velocity
        max_acc : float
            Maximum allowed acceleration
        duration : float
            Total trajectory duration
            
        Returns
        -------
        np.ndarray
            Array of shape (n_points, 7) with joint angles at 20ms intervals
            
        Hints
        -----
        - Use three phases: acceleration, constant velocity, deceleration
        - Calculate proper acceleration time
        - Ensure smooth transitions between phases
        """
        max_vel = 5
        max_acc = 5
        # Time interval
        dt = 0.02  # 20 ms intervals
        time_steps = np.arange(0, duration + dt, dt)
        n_points = len(time_steps)

        # Number of joints
        num_joints = q_start.shape[0]

        # Initialize trajectory array
        trajectory = np.zeros((n_points, num_joints))

        # Compute delta_q for each joint
        delta_q = q_end - q_start

        # Sign of movement for each joint
        sign = np.sign(delta_q)

        # Absolute distance to move for each joint
        dist = np.abs(delta_q)

        # Initialize arrays to store per-joint parameters
        a_j = np.zeros(num_joints)
        v_peak_j = np.zeros(num_joints)
        t_ramp_j = np.zeros(num_joints)
        q_j_t_ramp = np.zeros(num_joints)
        q_j_t_total_minus_t_ramp = np.zeros(num_joints)

        # Time available for acceleration/deceleration (assuming symmetric)
        t_total = duration
        t_ramp = t_total / 4

        # Compute per-joint required acceleration and peak velocity
        for j in range(num_joints):
            
            # Peak velocity
            v_peak_j[j] = dist[j]/ (t_total - t_ramp)
            if v_peak_j[j] > max_vel:
                raise ValueError(f"Peak velocity for joint {j} exceeds max_vel.")
            # Required acceleration
            a_j[j] = v_peak_j[j]/t_ramp
            if a_j[j] > max_acc:
                raise ValueError(f"Required acceleration for joint {j} exceeds max_acc.")
            
            # Positions at key times
            q_j_t_ramp[j] = q_start[j] + 0.5 * a_j[j] * t_ramp ** 2 * sign[j]
            q_j_t_total_minus_t_ramp[j] = q_j_t_ramp[j] + v_peak_j[j] * (t_total - 2 * t_ramp) * sign[j]

        # Generate trajectory
        for i, t in enumerate(time_steps):
            for j in range(num_joints):
                if t <= t_ramp:
                    # Acceleration phase
                    q = q_start[j] + 0.5 * a_j[j] * t ** 2 * sign[j]
                elif t <= t_total - t_ramp:
                    # Constant velocity phase
                    q = q_j_t_ramp[j] + v_peak_j[j] * (t - t_ramp) * sign[j]
                else:
                    # Deceleration phase
                    t_dec = t - (t_total - t_ramp)
                    q = q_j_t_total_minus_t_ramp[j] + v_peak_j[j] * t_dec * sign[j] - 0.5 * a_j[j] * t_dec ** 2 * sign[j]
                trajectory[i, j] = q

        return trajectory



    def interpolate_trajectory(self, times, trajectory):
        """
        Interpolate a given trajectory to match the required 20ms control rate.
        This function will create a new trajectory that aligns with the timing requirements
        for dynamic control of the robot.

        Parameters
        ----------
        times : array_like
            An array of time points for which the trajectory needs interpolation.
        trajectory : array_like
            An array of waypoints or joint configurations that define the initial trajectory.

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Notes
        -----
        - The output trajectory should have waypoints spaced at exactly 20ms intervals.
        - This method is critical for ensuring smooth motion and adhering to the control loop timing.

        Hints
        -----
        - Consider using linear interpolation for simple cases or cubic spline interpolation for smoother trajectories.
        - Ensure the interpolated trajectory respects the physical and operational limits of the robot, such as
        maximum velocities and accelerations.
        """
        raise NotImplementedError("Implement trajectory interpolation")

class TrajectoryFollower:
    def __init__(self):
        self.dt = 0.02  # Required 20ms control loop
        self.fa = FrankaArm()
        
    def follow_joint_trajectory(self, joint_trajectory):
        """
        Follow a joint trajectory using dynamic control.
        
        From writeup: Must have 20ms between waypoints and maintain smooth motion
        
        Parameters
        ----------
        joint_trajectory : np.ndarray
            Array of shape (N, 7) containing joint angles for each timestep
        """
        rospy.loginfo('Initializing Sensor Publisher')
        pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        rate = rospy.Rate(1 / self.dt)

        rospy.loginfo('Publishing joints trajectory...')
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(joint_trajectory[0], duration=1000, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(1, joint_trajectory.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=joint_trajectory[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            
            rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            pub.publish(ros_msg)
            rate.sleep()

        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
        pub.publish(ros_msg)

        rospy.loginfo('Done')
            
    def follow_cartesian_trajectory(self, pose_trajectory):
        """
        Follow a Cartesian trajectory using dynamic control.
        Must avoid collisions and handle pen tip offset.
        """
        joint_trajectory = []
        current_joints = self.robot.arm.get_joints()
        
        # Convert to joint trajectory
        for pose in pose_trajectory:
            # Get joint configuration
            target_joints = self._inverse_kinematics(pose, current_joints)
            if target_joints is None:
                raise Exception("Unable to find valid IK solution")
                
            # Check velocity limits
            if len(joint_trajectory) > 0:
                velocity = (target_joints - joint_trajectory[-1]) / self.dt
                if np.any(np.abs(velocity) > RobotConfig.MAX_VELOCITY):
                    raise Exception("Velocity limit exceeded")
                    
            joint_trajectory.append(target_joints)
            current_joints = target_joints
            
        # Execute trajectory
        self.follow_joint_trajectory(np.array(joint_trajectory))
