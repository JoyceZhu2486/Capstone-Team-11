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
            if i>=7: theta = 0
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
        epsilon = 1

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
            frames = np.zeros((4, 4, len(self.dh_parameters)+1))
            # for each frame
            for j in range(len(frames_less)):
                # computer the change in x, y, z, roll, pitch, yaw
                [x_l, y_l, z_l, roll_l, pitch_l, yaw_l] = \
                    self.frame_to_workspace(frames_less[:, :, j])
                [x_m, y_m, z_m, roll_m, pitch_m, yaw_m] = \
                    self.frame_to_workspace(frames_more[:, :, j])
    
                jacobians[0][i][j] = (x_m     - x_l    ) / epsilon
                jacobians[1][i][j] = (y_m     - y_l    ) / epsilon
                jacobians[2][i][j] = (z_m     - z_l    ) / epsilon
                jacobians[3][i][j] = (roll_m  - roll_l ) / epsilon
                jacobians[4][i][j] = (pitch_m - pitch_l) / epsilon
                jacobians[5][i][j] = (yaw_m   - yaw_l  ) / epsilon
        
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
        print("Hs[:,:,-1]", Hs[:,:,-1])
        print("on", on)
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
        # n = self.dof

        # if thetas.ndim != 1:
        #     raise ValueError('Expecting a 1D array of joint angles.')

        # if thetas.shape[0] != n:
        #     raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {n}')

        # # Initialize lists for z axes and origins
        # z = [np.array([0, 0, 1])]  # z0 axis
        # o = [np.array([0, 0, 0])]  # o0 origin

        # # Compute forward kinematics to get transformations
        # frames = self.forward_kinematics(thetas)
        # T = [frames[..., i] for i in range(frames.shape[2])]

        # for i in range(1, len(T)):
        #     z.append(T[i][:3, 2])
        #     o.append(T[i][:3, 3])

        # # Compute Jacobian
        # J = np.zeros((6, n))
        # o_n = o[-1]  # Position of the end-effector

        # for i in range(n):
        #     Jp = np.cross(z[i], (o_n - o[i]))
        #     Jo = z[i]
        #     J[:3, i] = Jp
        #     J[3:, i] = Jo

        # return J

    def _inverse_kinematics(self, target_pose, thetas, method):
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
        
        if thetas.shape[0] != (self.dof):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        if type(target_pose) == RigidTransform:
            # raise ValueError('Invalid target_pose: Expected RigidTransform.')
            transfer_matrix = np.eye(4)
            for i in range(3):
                for j in range(3):
                    transfer_matrix[i][j] = target_pose.rotation[i][j]
                transfer_matrix[i][3] = target_pose.translation[i]
        else:
            transfer_matrix = target_pose

        if thetas is None:
            thetas = self.fa.get_joints()

        #print("transfer_matrix:",transfer_matrix)
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO: Implement gradient inverse kinematics
        #  Motion planning parameters
        ##PATH_RESOLUTION = 0.01  # meters
        
        # Step size for gradient update
        step_size = 0.05

        ##IK_TOLERANCE = 1e-3#
        
        num_iter = 1
        
        pcos =0
        # Run gradient descent optimization
        while num_iter < TaskConfig.IK_MAX_ITERATIONS +2000:
            # TODO:[x, y, z, roll, pitch, yaw] Check if this is the right type for target_pose
            
            cost_gradient = np.zeros(self.dof)
            # Compute the current end effector pose
            if (method):#this method should work
                hfk = self.forward_kinematics(thetas)[:,:,-1]
                rfk = hfk[:3,:3]
                rtp = transfer_matrix[:3,:3]
                axisR = rfk.T @ rtp
                robj = R.from_matrix(axisR)
                rotvec = robj.as_rotvec()# in the world frame/the frame rfk and rtp referenced to
                rotvec = rfk @ rotvec #do we need to time a rotation
                print("rotvec", rotvec)
                print("(transfer_matrix-hfk)[:3,-1]", (transfer_matrix-hfk)[:3,-1])
                #might need to change theta to row pitch yaw convention, by reverse the sequence
                #rotvec = rotvec[::-1] 
                #print("(ntransfer_matrix-hfk)[:3,-1]", (transfer_matrix-hfk)[:3,-1])
                #print("rotv", rotvec)
                d = np.concatenate(((hfk-transfer_matrix)[:3,-1], rotvec))
                print("d", d)

            else:#this might also work
                FK = self.end_effector(thetas)
                TP = self.end_effector(transfer_matrix)
                # Compute the difference between current pose and goal
                d = FK - TP
            
            d[3:] = 0 #mask the rotation
            #print("d", d)
            # Compute the Jacobian
            J = self.analy_jacobian(thetas)
            print("J", J)
            # Compute the cost gradient
            cost_gradient = np.dot(J.T, d)
            print("cost_gradient ",cost_gradient )
            thetas = thetas - step_size * cost_gradient

            for theta in thetas:
                if abs(theta)>= 2*np.pi:
                    theta += -np.sign(theta)*2*np.pi
            # Check stopping condition, and return if it is met.
            print("difference", np.linalg.norm(cost_gradient)-pcos)
            print("np.linalg.norm(cost_gradient)",np.linalg.norm(cost_gradient))
            if np.linalg.norm(cost_gradient) < TaskConfig.IK_TOLERANCE :
                print("Reached")
                return thetas
            pcos = np.linalg.norm(cost_gradient)
            num_iter += 1

        print("Reached Max Iteration")
        return thetas