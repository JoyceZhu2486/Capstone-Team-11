from frankapy import FrankaArm
import numpy as np
from autolab_core import RigidTransform

class WorkspaceCalibrator:
    def __init__(self):
        self.fa = FrankaArm()
        self.duration = 20
        
    def calibrate_pen_holders(self):
        """Calibrate pen holder positions through guided movement"""
        print("\nCalibrating pen holder...")
        print("Moving to home position...")
        self.fa.reset_joints()
        

        input(f"Press Enter to calibrate pen holder")
        print(f"Move robot above a pen, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)  # Allow manual positioning
        
        # Record position
        current_pose = self.fa.get_pose()
        print(f"Recorded pen holder at: {current_pose.translation}")
        np.save("pen_holder_pose.npy", current_pose.translation)
        return current_pose
    
    def calibrate_whiteboard(self):
        """Calibrate whiteboard position and orientation"""
        print("\nCalibrating whiteboard...")
        print("Moving to home position...")
        self.fa.reset_joints()
        
        # Get three points to define whiteboard plane
        points = []
        for i in range(3):
            if i == 0:
                input(f"Press Enter to record whiteboard origin")
            else:
                input(f"Press Enter to record whiteboard point {i+1}")
            print(f"You can proceed after {self.duration} seconds")
            self.fa.run_guide_mode(duration=self.duration)
            current_pose = self.fa.get_pose()
            points.append(current_pose.translation)
            print(f"Recorded point {i+1} at {current_pose.translation}")
            
        # # Calculate whiteboard plane and orientation
        p1, p2, p3 = points
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)
        
        # Create whiteboard frame
        rotation = self._compute_orientation_matrix(normal)
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = p1
        print(f"Recorded whiteboard at: {T}")
        np.save("whiteboard_pose.npy", T)
        
        return current_pose
    
    def calibrate_drop_location(self):
        """Calibrate pen drop location"""
        print("\nCalibrating drop location...")
        print("Moving to home position...")
        self.fa.reset_joints()
        input("Press Enter to calibrate drop location")
        print(f"Move robot above the drop bin, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)
        drop_pose = self.fa.get_pose()
        print(f"Recorded drop bin at: {drop_pose.translation}")
        np.save("drop_bin_pose.npy", drop_pose.translation)
        return drop_pose
    
    def _compute_orientation_matrix(self, normal):
        """Compute rotation matrix from normal vector"""
        z_axis = normal
        x_axis = np.array([1, 0, 0])
        if abs(np.dot(z_axis, x_axis)) > 0.9:
            x_axis = np.array([0, 1, 0])
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        return np.column_stack((x_axis, y_axis, z_axis))

def main():
    calibrator = WorkspaceCalibrator()
    
    HOMEJOINTS = np.array([-3.91461865e-04, -7.85867822e-01, 1.17114550e-04, -2.35666438e+00,
        -6.39495011e-04,  1.57148656e+00,  7.85797890e-01])
    # Perform calibration
    pen_positions = calibrator.calibrate_pen_holders()
    calibrator.fa.goto_joints(HOMEJOINTS)
    input("Press enter to close gripper")
    calibrator.fa.close_gripper()
    whiteboard_pose = calibrator.calibrate_whiteboard()
    drop_pose = calibrator.calibrate_drop_location()
    calibrator.fa.open_gripper()

if __name__ == "__main__":
    main()
