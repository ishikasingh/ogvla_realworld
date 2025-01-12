import numpy as np


# Note, the order of quaternion for franka panda is x,y,z,w


def quaternion_to_rpy(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion  # Adjusted for x, y, z, w order

    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Clamp to [-1, 1] to handle potential numerical errors
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    euler_angles = np.array([roll, pitch, yaw])

    return euler_angles


def rpy_to_quaternion(euler: np.ndarray) -> np.ndarray:
    r, p, y = euler
    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)

    # Compute quaternion components and adjust for x, y, z, w order
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    quaternion = np.array([qx, qy, qz, qw])

    return quaternion


def quaternion_inverse(quaternion: np.ndarray) -> np.ndarray:
    # For a quaternion [x, y, z, w], the inverse is [-x, -y, -z, w] / (x^2 + y^2 + z^2 + w^2) for non-normalized quaternions
    norm_squared = np.dot(quaternion, quaternion)
    return np.array([-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]) / norm_squared


def quaternion_multiply_translation(quaternion: np.ndarray, translation: np.ndarray) -> np.ndarray:
    # Apply quaternion rotation to a translation vector
    x, y, z, w = quaternion
    tx, ty, tz = translation

    # Compute the rotated translation vector
    t2 = np.array([
        2.0 * (y * tz - z * ty),
        2.0 * (z * tx - x * tz),
        2.0 * (x * ty - y * tx)
    ])
    rotated_translation = translation + w * t2 + np.cross([x, y, z], t2)

    return rotated_translation


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiplies two quaternions.

    Args:
        q1 (np.ndarray): First quaternion as [x, y, z, w].
        q2 (np.ndarray): Second quaternion as [x, y, z, w].

    Returns:
        np.ndarray: Resulting quaternion from the multiplication [x, y, z, w].
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    # Quaternion multiplication formula
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return np.array([x, y, z, w])


def rpy_to_rotation_matrix(rpy: np.ndarray) -> np.ndarray:
    """
    Converts Roll-Pitch-Yaw (RPY) angles to a 3x3 rotation matrix.

    Args:
        rpy (np.ndarray): Array of RPY angles [roll, pitch, yaw].

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    roll, pitch, yaw = rpy

    # Rotation matrix around the x-axis (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Rotation matrix around the y-axis (pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation matrix around the z-axis (yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combine the rotations in ZYX order (yaw, pitch, roll)
    rotation_matrix = R_z @ R_y @ R_x

    return rotation_matrix


def quaternion_slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    """
    Performs spherical linear interpolation (SLERP) between two quaternions.

    Args:
        q1 (np.ndarray): Starting quaternion as [x, y, z, w].
        q2 (np.ndarray): Target quaternion as [x, y, z, w].
        t (float): Interpolation factor, between 0 and 1.

    Returns:
        np.ndarray: Interpolated quaternion as [x, y, z, w].
    """
    # Normalize input quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute the dot product (cosine of the angle)
    dot_product = np.dot(q1, q2)

    # If the dot product is negative, slerp will not take the shorter path.
    # So we invert one quaternion to ensure the shortest path is taken.
    if dot_product < 0.0:
        q2 = -q2
        dot_product = -dot_product

    # Clamp the dot product to avoid numerical errors in acos
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # Calculate the angle between the quaternions
    theta_0 = np.arccos(dot_product)  # Original angle
    theta = theta_0 * t  # Scaled angle

    # Compute the second quaternion orthogonal to q1
    q2_orthogonal = q2 - q1 * dot_product
    q2_orthogonal /= np.linalg.norm(q2_orthogonal)

    # Interpolate
    interpolated_quat = q1 * np.cos(theta) + q2_orthogonal * np.sin(theta)

    return interpolated_quat
