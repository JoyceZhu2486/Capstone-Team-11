import sys
sys.path.append("../config")

import numpy as np
from autolab_core import RigidTransform
from robot_config import RobotConfig

class TrajectoryGenerator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.max_vel = RobotConfig.MAX_VELOCITY
        self.max_acc = RobotConfig.MAX_ACCELERATION
    
    def generate_pose_waypoints(self, start_pose, end_pose, duration):
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
        num_points = duration/0.02
        pose_trajectory = np.linspace(start_pose, end_pose, num_points).T
        return pose_trajectory

    def generate_cartesian_waypoints(self, start_pose, end_pose, duration):
        """
        Generate a Cartesian trajectory as a series of waypoints using linear interpolation 
        for position and Spherical Linear Interpolation (SLERP) for orientation.
        
        This method calculates waypoints at regular intervals over the specified duration
        to create a smooth motion from the starting pose to the ending pose. Each waypoint
        includes both position and orientation data, ensuring that the trajectory is not only
        spatially accurate but also maintains correct alignment throughout the movement.

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
        list of RigidTransform
            A list of RigidTransform objects representing the waypoints of the trajectory.
            Each waypoint is spaced at an interval of 20ms, as defined by `self.dt`.

        Notes
        -----
        - Waypoints are calculated at 20ms intervals.
        - Linear interpolation is used for the translational component of the trajectory.
        - SLERP is utilized for the rotational component to ensure smooth transitions between orientations.
        - The trajectory generation assumes constant velocity, which may not be suitable for all applications
        where acceleration and deceleration phases are required.

        Hints
        -----
        - The number of points, `n_points`, is determined by the duration divided by the timestep `self.dt`.
        - Use `np.linspace` to generate times from 0 to `duration` for the waypoints.
        - For position, interpolate linearly between `start_pose.translation` and `end_pose.translation`.
        - For rotation, convert rotation matrices to quaternions and use SLERP from `q0` to `q1`.
        - Ensure each waypoint is constructed with the interpolated position and orientation and respects
        the frame specifications of the `start_pose`.
        """
        raise NotImplementedError("Implement cartesian waypoints generation")
    
    def generate_straight_line(self, start_point, end_point, duration=None):
        """
        This function creates a smooth straight-line trajectory for the robot's end-effector to follow.

        Parameters
        ----------
        start_point : array_like
            The starting point of the line in Cartesian space.
        end_point : array_like
            The ending point of the line in Cartesian space.
        duration : float, optional
            The total duration over which the line should be drawn. If not specified,
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
        - The method needs to handle both the translational and rotational dynamics to ensure
        that the end-effector is properly aligned during the motion.
        - If duration is not provided, calculate it based on a default speed and the distance between points.

        Hints
        -----
        - Use linear interpolation to compute intermediate points along the line.
        - Optionally, incorporate orientation interpolation if the end-effector's orientation is critical.
        - This method should eventually call a function to actually move the robot's joints based on the
        interpolated points.
        """
        raise NotImplementedError("Implement line drawing trajectory")
        
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
    
    def generate_trapezoidal_trajectory(self, q_start, q_end, max_vel, max_acc, duration):
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
        t_ramp = t_total / 2

        # Compute per-joint required acceleration and peak velocity
        for j in range(num_joints):
            # Required acceleration
            a_j[j] = (4 * dist[j]) / (t_total ** 2)
            if a_j[j] > max_acc:
                raise ValueError(f"Required acceleration for joint {j} exceeds max_acc.")
            
            # Peak velocity
            v_peak_j[j] = a_j[j] * t_ramp
            if v_peak_j[j] > max_vel:
                raise ValueError(f"Peak velocity for joint {j} exceeds max_vel.")
            
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
    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.dt = 0.02  # Required 20ms control loop
        
    def follow_joint_trajectory(self, joint_trajectory):
        """
        Follow a joint trajectory using dynamic control.
        
        From writeup: Must have 20ms between waypoints and maintain smooth motion
        
        Parameters
        ----------
        joint_trajectory : np.ndarray
            Array of shape (N, 7) containing joint angles for each timestep
        """
        # Interpolate trajectory to match control rate
        times = np.linspace(0, len(joint_trajectory) * self.dt, len(joint_trajectory))
        joint_trajectory_interp = self.interpolate_trajectory(times, joint_trajectory)
        
        # Execute trajectory using dynamic control
        self.robot.arm.goto_joints(
            joints=joint_trajectory_interp[0],  # Initial joint position
            dynamic=True,
            buffer_time=len(joint_trajectory_interp) * self.dt
        )
        
        for joints in joint_trajectory_interp[1:]:
            self.robot.arm.goto_joints(
                joints=joints,
                duration=self.dt,
                dynamic=True,
                buffer_time=self.dt
            )
            
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