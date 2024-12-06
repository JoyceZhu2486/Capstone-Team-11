import sys
sys.path.append('../config')
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from robot_config import RobotConfig
from task_config import TaskConfig
from scipy.spatial.transform import Rotation as R

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7
        self.arm = FrankaArm()
        self.dh_parameters = np.array( [[0,0,0.333,0],
                                        [0,-np.pi/2,0,0],
                                        [0,np.pi/2,0.316,0],
                                        [0.0825,np.pi/2,0,0],
                                        [-0.0825,-np.pi/2,0.384,0],
                                        [0,np.pi/2,0,0],
                                        [0.088,np.pi/2,0,0],
                                        [0,0,0.107,0],
                                        [0,0,0.1034,0]])

    def dh_transformation(self, a, alpha, d, theta):
        """Compute the individual transformation matrix using DH parameters."""
        # Rot_x_alpha = np.array([
        #     [1, 0, 0, 0],
        #     [0, np.cos(alpha), -np.sin(alpha), 0],
        #     [0, np.sin(alpha), np.cos(alpha), 0],
        #     [0, 0, 0, 1]
        # ])

        # Trans_x_a = np.array([
        #     [1, 0, 0, a],
        #     [0, 1, 0, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1]
        # ])

        # Trans_z_d = np.array([
        #     [1, 0, 0, 0],
        #     [0, 1, 0, 0],
        #     [0, 0, 1, d],
        #     [0, 0, 0, 1]
        # ])

        # Rot_z_theta = np.array([
        #     [np.cos(theta), -np.sin(theta), 0, 0],
        #     [np.sin(theta), np.cos(theta), 0, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1]
        # ])

        # T_joint = Rot_x_alpha @ Trans_x_a @ Trans_z_d @ Rot_z_theta 
        #T_joint = Rot_z_theta @ Trans_z_d @ Trans_x_a @ Rot_x_alpha
        #T_joint = Rot_x_alpha@ Trans_x_a @ Trans_z_d @ Rot_z_theta 
        #T_joint = Trans_x_a  @ Trans_z_d @ Rot_x_alpha@ Rot_z_theta 

        T_joint = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
            [0, 0, 0, 1]
        ])


        return T_joint

    def forward_kinematics(self, thetas):
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
        # Initialize frames array: 4x4 transformation matrices for all frames
        frames = np.zeros((4, 4, len(self.dh_parameters)+1))
        frames[..., 0] = np.eye(4)  # Base frame (identity matrix)

        end_effector_pose = np.eye(4)

        for i in range(len(self.dh_parameters)):
            a, alpha, d, theta_offset = self.dh_parameters[i]
            if i==7: theta = -np.pi/4
            elif i == 8: theta = 0
            else:
                theta = thetas[i] + theta_offset  # Add the offset to the joint angle
            
            # Compute the individual transformation matrix for each joint
            T_joint = self.dh_transformation(a, alpha, d, theta)
            
            # Store the current transformation in frames
            frames[..., i + 1] = frames[..., i] @ T_joint


        return frames
        # --------------- END STUDENT SECTION --------------------------------------------------

    def end_effector(self, thetas):
        """
        Compute the end-effector pose and convert it into workspace configuration.

        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters, where each row is [a, alpha, d, theta_offset].
        thetas: np.ndarray
            Joint angles for the robot.

        Returns
        -------
        np.ndarray
            Workspace configuration of the end-effector in [x, y, z, roll, pitch, yaw] format.
        """
        # Compute forward kinematics to get the end-effector frame
        end_effector_frame = self.forward_kinematics(thetas)
        end_effector_frame = end_effector_frame[...,-1]
        
        # Convert the end-effector frame to workspace configuration
        workspace_configuration = self.frame_to_workspace(end_effector_frame)
        
        return np.array(workspace_configuration)


    def frame_to_workspace(self, frame):
        """
        Compute workspace variables given a frame.

        Parameters
        ----------
        frame : np.array[4, 4]
            the transformation frame 

        Returns
        -------
        array: workspace variables
            in [x, y, z, roll, pitch, yaw] format.

        """
        # obtain x, y, z
        x = frame[0, 3]
        y = frame[1, 3]
        z = frame[2, 3]
        # calculate pitch, roll, yaw
        # Check if we are in a gimbal lock position
        if abs(frame[2, 0]) != 1:
            # Normal case
            pitch = -np.arcsin(frame[2, 0])
            roll = np.arctan2(frame[2, 1], 
                                frame[2, 2])
            yaw = np.arctan2(frame[1, 0], 
                                frame[0, 0])
        else:
            # Gimbal lock case
            yaw = 0  # Set yaw to zero in the gimbal lock case
            if frame[2, 0] == -1:
                pitch = np.pi / 2
                roll = np.arctan2(frame[0, 1], 
                                    frame[0, 2])
            else:
                pitch = -np.pi / 2
                roll = np.arctan2(-frame[0, 1], 
                                    -frame[0, 2])
        return [x, y, z, roll, pitch, yaw]

    def jacobian(self, thetas):
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

        jacobians = np.zeros((6, self.dof, self.dof + 2))
        epsilon = 0.001

        # --------------- BEGIN STUDENT SECTION ----------------------------------------
        # TODO: Implement the numerical computation of the Jacobians

        # Hints:
        # - Perturb each joint angle by epsilon and compute the forward kinematics.
        # - Compute numerical derivatives for x, y, and positions.
        # - Determine the rotational component based on joint contributions.

        # for each joint change
        poses = self.forward_kinematics(thetas)
        # print("base poses is:", poses)

        for i in range(self.dof):
            # thetas_less = np.copy(thetas)
            # thetas_less[i] = thetas_less[i] - epsilon
            # frames_less = self.forward_kinematics(thetas_less)

            thetas_more = np.copy(thetas)
            thetas_more[i] += epsilon
            frames_more = self.forward_kinematics(thetas_more)
            
            # for each frame
            for frame in range(self.dof + 2):
                base_pose = poses[...,frame]
                # print("base_pose is:", base_pose)
                frame_more = frames_more[...,frame]

                # Compute translational component
                delta_p = (frame_more[:3, 3] - base_pose[:3, 3]) / epsilon
                jacobians[:3, i, frame] = delta_p

                # Compute rotational component
                delta_R = frame_more[:3, :3] @ base_pose[:3, :3].T
                rotation = R.from_matrix(delta_R)
                delta_omega = rotation.as_rotvec() / epsilon
                jacobians[3:, i, frame] = delta_omega

        return jacobians
        # --------------- END STUDENT SECTION --------------------------------------------
    
    
    def analy_jacobian (self, thetas):
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
        #frames = np.zeros((4, 4, len(self.dh_parameters)+1)) 10 frames
        Hs = self.forward_kinematics(thetas)
        on = Hs[:3,3,-1]
        # print("Hs[:,:,-1]", Hs[:,:,-1])
        # print("on", on)
        Jv = np.zeros((3, self.dof))  # Linear velocity part
        Jw = np.zeros((3, self.dof))  # Angular velocity part
        # for each joint change
        for i in range(self.dof):
            if(i == self.dof -1):
                o = on - Hs[:3,3,self.dof -2]
                z = Hs[:3,2,-1]
            else:
                o = on - Hs[:3,3,i]
                z = Hs[:3,2,i]
            Jv[:,i] = np.cross(z, o)
            Jw[:,i] = z

        J = np.vstack((Jv,Jw))
        
        return J


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
        
        if seed_joints.shape[0] != (self.dof):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        if type(target_pose) == RigidTransform:
            target_matrix = target_pose.matrix
        else:
            target_matrix = target_pose
            
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO: Implement gradient inverse kinematics
        max_iterations = TaskConfig.IK_MAX_ITERATIONS
        tolerance = TaskConfig.IK_TOLERANCE
        alpha = 0.05
        
        current_joints = seed_joints
        for _ in range(max_iterations):
            current_pose = self.forward_kinematics(current_joints)
            pose_error = target_matrix - current_pose[...,-1]
            position_error = pose_error[:3, 3]
            # print((target_pose.rotation - current_pose[:3, :3]).flatten())
            #rotation_error = self._compute_rotation_error(target_pose.rotation, current_pose[:3, :3])
            # rotation_error = self._rotation_to_quaternion(target_pose.rotation)[1:] - self._rotation_to_quaternion(current_pose[:3, :3])[1:]
            print(target_matrix)
            rotation_error = (target_matrix[:3,:3]) @ current_pose[...,-1][:3,:3].T 
            rotation_error = R.from_matrix(rotation_error).as_rotvec()
            error = np.hstack((position_error, rotation_error))
            if np.linalg.norm(error) < tolerance:
                return current_joints
            
            jacobian = self.jacobian(current_joints)[:, :, -1]
            jacobian_pseudo_inverse = np.linalg.pinv(jacobian)
            delta_joints = alpha * jacobian_pseudo_inverse @ error
            current_joints += delta_joints
            
            # if not self.is_joints_reachable(current_joints):
            #     return None
        
        return None
        # --------------- END STUDENT SECTION ----------------


    def _rotation_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                qw = (R[2,1] - R[1,2]) / S
                qx = 0.25 * S
                qy = (R[0,1] + R[1,0]) / S
                qz = (R[0,2] + R[2,0]) / S
            elif R[1,1] > R[2,2]:
                S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                qw = (R[0,2] - R[2,0]) / S
                qx = (R[0,1] + R[1,0]) / S
                qy = 0.25 * S
                qz = (R[1,2] + R[2,1]) / S
            else:
                S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                qw = (R[1,0] - R[0,1]) / S
                qx = (R[0,2] + R[2,0]) / S
                qy = (R[1,2] + R[2,1]) / S
                qz = 0.25 * S
        return np.array([qw, qx, qy, qz])

