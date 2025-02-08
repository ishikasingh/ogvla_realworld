import numpy as np
from arm_hand_deployment.consts import FRANKA_EE_TO_ALLEGRO
from scipy.spatial.transform import Rotation as R

def get_rotation_matrix(roll, pitch, yaw):
    """
    Compute the rotation matrix from EE frame to world frame.
    
    Args:
        roll: Rotation about X-axis.
        pitch: Rotation about Y-axis.
        yaw: Rotation about Z-axis.
    
    Returns:
        3x3 Rotation matrix (EE → world).
    """
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def transform_gravity_vector(ee_pose):
    """
    Computes the gravity vector in the hand's base_link frame.

    Args:
        ee_pose: 6D end-effector pose [x, y, z, roll, pitch, yaw] from `GetEndEffectorPose()`.
    
    Returns:
        Gravity vector in the hand's frame.
    """
    # Extract roll, pitch, yaw from EE pose
    roll, pitch, yaw = ee_pose[3:]

    # Compute rotation matrix (EE → world)
    R_ee_to_world = get_rotation_matrix(roll, pitch, yaw)

    # Transform world gravity vector into EE frame
    g_world = np.array([0, 0, -9.81])  # Gravity in world frame
    g_ee = np.linalg.inv(R_ee_to_world) @ g_world  # Transform to EE frame

    # === Apply Correction Transformation from EE frame to Hand's base_link frame ===
    # When EE pose is [_, _, _, np.pi, 0, -np.pi/4], hand's frame should be world-aligned.
    # So we define this "default" rotation matrix:
    R_ee_to_hand = get_rotation_matrix(
        FRANKA_EE_TO_ALLEGRO[0], 
        FRANKA_EE_TO_ALLEGRO[1], 
        FRANKA_EE_TO_ALLEGRO[2]
    )

    # Apply correction: transform g_ee into the hand's base_link frame
    g_hand = np.linalg.inv(R_ee_to_hand) @ g_ee  

    return g_hand

# Example case: EE pose where hand should align with world gravity
ee_pose = np.array([0, 0, 0, np.pi, 0, -np.pi/4])  
g_hand = transform_gravity_vector(ee_pose)

print("Gravity in hand frame:", g_hand)  # Should output approximately [0, 0, -9.81]

# Example for an arbitrary EE pose
ee_pose_arbitrary = np.array([0, 0, 0, np.pi/2, np.pi/4, 0])  
g_hand_arbitrary = transform_gravity_vector(ee_pose_arbitrary)

print("Gravity in hand frame (arbitrary pose):", g_hand_arbitrary)