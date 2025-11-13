#!/usr/bin/env python3
from typing import Union, Literal
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3


def rotate_ENU_NED(array: np.ndarray) -> np.ndarray:
    """
    Converts coordinates between ENU and NED frames (and vice versa).

    :param array: Input vector in ENU format as np.ndarray [x, y, z].
    :return: Vector in NED format as np.ndarray [x, y, z].
    """
    if array.shape != (3,):
        raise ValueError("Input ENU vector must be a 1D array of length 3.")

    ned = np.array([array[1], array[0], -array[2]])
    return ned


def rotated_ENU_NED_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion between ENU and NED coordinate systems (and vice versa).

    :param q: Quaternion in format np.array([x, y, z, w]).
    :return: Converted quaternion in format np.array([x, y, z, w]).
    """
    if q.shape != (4,):
        raise ValueError("Quaternion must be an array with 4 elements.")
    q = np.array([q[1], q[0], -q[2], q[3]])
    return q


def transform_coordinates(
    source_position: np.ndarray,
    reference_position: np.ndarray,
    reference_orientation: Union[np.ndarray, list],
    transform_type: Literal["local_to_global", "global_to_local"],
    rotation_type: Literal["euler", "quaternion"] = "euler",
    sequence: str = "zyx",
) -> np.ndarray:
    """
    Transforms coordinates between local and global reference frames.

    :param source_position: Position to transform [x, y, z].
    :param reference_position: Origin of the reference frame in the other system [x, y, z].
    :param reference_orientation:
        - If rotation_type='euler': [yaw, pitch, roll] in radians.
        - If rotation_type='quaternion': [x, y, z, w].
    :param transform_type: 'local_to_global' or 'global_to_local'.
    :param rotation_type: 'euler' (default) or 'quaternion'.
    :param sequence: Euler angle rotation order (default 'zyx').
    :return: Transformed position as np.ndarray [x, y, z].
    """
    if rotation_type == "euler":
        rotation = R.from_euler(sequence, reference_orientation)
    elif rotation_type == "quaternion":
        rotation = R.from_quat(reference_orientation)
    else:
        raise ValueError(
            f"Unsupported rotation type: {rotation_type}. Available: 'euler', 'quaternion'."
        )

    if transform_type == "local_to_global":
        rotated_position = rotation.apply(source_position)
        transformed_position = rotated_position + reference_position
    elif transform_type == "global_to_local":
        delta_position = source_position - reference_position
        transformed_position = rotation.inv().apply(delta_position)
    else:
        raise ValueError(
            f"Unsupported transform type: {transform_type}. "
            "Available: 'local_to_global', 'global_to_local'."
        )
    return transformed_position


def transform_orientation(
    source_orientation: np.ndarray,
    reference_orientation: np.ndarray,
    transform_type: Literal["local_to_global", "global_to_local"],
    mode: Literal["quaternion", "euler"] = "quaternion",
    euler_order: str = "zyx"
) -> np.ndarray:
    """
    Transforms orientation between local and global coordinate frames.

    Parameters
    ----------
    source_orientation : np.ndarray
        Orientation to transform.
        - quaternion mode: [x, y, z, w]
        - euler mode: [roll, pitch, yaw]
    reference_orientation : np.ndarray
        Reference frame orientation (same format as source).
    transform_type : Literal
        - "local_to_global": local → global
        - "global_to_local": global → local
    mode : Literal
        "quaternion" or "euler"
    euler_order : str
        Euler rotation order (default "zyx")

    Returns
    -------
    np.ndarray
        Transformed orientation in the same format as input.
    """
    if mode not in ["quaternion", "euler"]:
        raise ValueError(f"Unsupported mode: {mode}. Available: 'quaternion', 'euler'.")

    # Convert to Rotation objects
    if mode == "quaternion":
        source_rotation = R.from_quat(source_orientation)
        reference_rotation = R.from_quat(reference_orientation)
    elif mode == "euler":
        source_rotation = R.from_euler(euler_order, source_orientation)
        reference_rotation = R.from_euler(euler_order, reference_orientation)

    # Apply transformation
    if transform_type == "local_to_global":
        transformed_rotation = reference_rotation * source_rotation
    elif transform_type == "global_to_local":
        transformed_rotation = reference_rotation.inv() * source_rotation
    else:
        raise ValueError(
            f"Unsupported transform type: {transform_type}. "
            "Available: 'local_to_global', 'global_to_local'."
        )

    # Convert back to desired format
    if mode == "quaternion":
        return transformed_rotation.as_quat()
    else:
        return transformed_rotation.as_euler(euler_order)


def calculate_yaw_towards_target(current_position: Vector3, target_position: Vector3) -> float:
    """
    Calculates yaw (in ENU) from current position to target position.

    :param current_position: Current position (geometry_msgs/Vector3).
    :param target_position: Target position (geometry_msgs/Vector3).
    :return: Yaw angle in radians (ENU convention).
    """
    delta_x = target_position.x - current_position.x
    delta_y = target_position.y - current_position.y
    yaw_ENU = np.arctan2(delta_y, delta_x)
    return float(yaw_ENU)


def calculate_distance(array1: Union[np.ndarray, list], array2: Union[np.ndarray, list]) -> float:
    """
    Computes Euclidean distance between two arrays of any dimension.

    :param array1: First coordinate array or list.
    :param array2: Second coordinate array or list.
    :return: Euclidean distance.
    :raises ValueError: If arrays have different shapes.
    """
    array1 = np.array(array1)
    array2 = np.array(array2)

    if array1.shape != array2.shape:
        raise ValueError("Input arrays must have the same shape.")

    return float(np.linalg.norm(array1 - array2))
