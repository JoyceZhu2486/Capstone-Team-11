import sys
sys.path.append('../config')
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from robot_config import RobotConfig
from task_config import TaskConfig

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 5

    def dh_transformation(self, a, alpha, d, theta):
        """Compute the individual transformation matrix using DH parameters."""
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        
    def forward_kinematcis(self, dh_parameters, thetas):
        """
        Compute foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO
        frames = np.zeros((4, 4, len(dh_parameters)+1))
        frames[..., 0] = np.eye(4)

        end_effector_pose = np.eye(4)

        for i in range(self.dof):
            a, alpha, d, _ = dh_parameters[i]
            theta = thetas[i]
            
            # Compute the individual transformation matrix for each joint
            transform = self.dh_transformation(a, alpha, d, theta)
            
            # Store the current transformation in frames
            frames[..., i + 1] = frames[..., i] @ transform
        
        return frames
        # --------------- END STUDENT SECTION --------------------------------------------------

    def end_effector(self, thetas):
        """
        Returns [x; y; z; roll; pitch; yaw] for the end effector given a set of joint angles.

        Parameters:
        thetas (np.ndarray): 1D array of joint angles.

        Returns:
        np.ndarray: End-effector position and orientation in [x, y, z, roll, pitch, yaw] format.
        """
        # Get the frames for all joints and the end-effector
        frames = self.fk(thetas)
        H_0_ee = frames[..., -1]

        # Extract end-effector position
        x = H_0_ee[0, 3]
        y = H_0_ee[1, 3]
        z = H_0_ee[2, 3]

        # Extract end-effector orientation (roll, pitch, yaw)
        roll = np.arctan2(H_0_ee[2, 1], H_0_ee[2, 2])
        pitch = np.arctan2(-H_0_ee[2, 0], np.sqrt(H_0_ee[2, 1]**2 + H_0_ee[2, 2]**2))
        yaw = np.arctan2(H_0_ee[1, 0], H_0_ee[0, 0])

        return np.array([x, y, z, roll, pitch, yaw])
        
    
    def jacobians(self, thetas):
        """
        Compute the Jacobians for each frame.

        Parameters
        ----------
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            Jacobians
        """
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        jacobians = np.zeros((6, self.dof, self.dof + 1))
        epsilon = 0.001

        # --------------- BEGIN STUDENT SECTION ----------------------------------------
        # TODO: Implement the numerical computation of the Jacobians

        # Hints:
        # - Perturb each joint angle by epsilon and compute the forward kinematics.
        # - Compute numerical derivatives for x, y, and positions.
        # - Determine the rotational component based on joint contributions.

        raise NotImplementedError("Implement jacobians")
        # --------------- END STUDENT SECTION --------------------------------------------
    
    
    def _inverse_kinematics(self, target_pose, seed_joints):
        """
        Compute inverse kinematics using Jacobian pseudo-inverse method.
        
        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution
        
        Parameters
        ----------
        target_pose : RigidTransform
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() and get_jacobian() from robot arm
        - Use _compute_rotation_error() for orientation error
        - Check joint limits with is_joints_reachable()
        - Track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig
        """
        
        if seed_joints.shape != (self.dof):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        if type(target_pose) != RigidTransform:
            raise ValueError('Invalid target_pose: Expected RigidTransform.')
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO: Implement gradient inverse kinematics

        raise NotImplementedError("Implement inverse kinematics")
        # --------------- END STUDENT SECTION --------------------------------------------------