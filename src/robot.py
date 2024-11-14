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
        self.dof = 7
    
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
        
        raise NotImplementedError("Implement forward kinematics")
        # --------------- END STUDENT SECTION --------------------------------------------------
    
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

        # for each joint change
        for i in range(self.dof):
            thetas_less = thetas
            thetas_less[i] = thetas_less[i] - epsilon
            frames_less = self.forward_kinematics(thetas_less)
            
            thetas_more = thetas
            thetas_more[i] = thetas_more[i] + epsilon
            frames_more = self.forward_kinematics(thetas_more)

            # for each frame
            for j in range(self.dof+1):
                # computer the change in x, y, z, \theta, \phi, \psi
                x_less = frames_less[:, :, j][0, 2]
                y_less = frames_less[:, :, j][1, 2]
                z_less = frames_less[:, :, j][2, 2]
                th_less = np.arctan2(frames_less[:, :, j][1, 0],
                                     frames_less[:, :, j][0, 0])
                
                

                x_more = frames_more[:, :, j][0, 2]
                y_more = frames_more[:, :, j][1, 2]
                z_more = frames_more[:, :, j][2, 2]
                th_more = np.arctan2(frames_more[:, :, j][1, 0],
                                     frames_more[:, :, j][0, 0])
                
                jacobians[0][i][j] = (x_more-x_less)/(epsilon)
                jacobians[1][i][j] = (y_more-y_less)/(epsilon)
                jacobians[2][i][j] = (th_more-th_less)/(epsilon)
        # Your code ends here
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
