import math
import numpy as np
from numpy.typing import NDArray


def normalize(vector: NDArray[np.float32]) -> NDArray[np.float32]:
    """_summary_

    Args:
        vector (NDArray[np.float32]): _description_

    Returns:
        NDArray[np.float32]: _description_
    """
    if np.linalg.norm(vector) == 0:
        return vector

    return vector / np.linalg.norm(vector)


def rotation_xyz(pointcloud, theta_x, theta_y, theta_z):
    theta_x = math.radians(theta_x)
    theta_y = math.radians(theta_y)
    theta_z = math.radians(theta_z)
    rot_x = np.array(
        [
            [1, 0, 0],
            [0, math.cos(theta_x), -math.sin(theta_x)],
            [0, math.sin(theta_x), math.cos(theta_x)],
        ]
    )

    rot_y = np.array(
        [
            [math.cos(theta_y), 0, math.sin(theta_y)],
            [0, 1, 0],
            [-math.sin(theta_y), 0, math.cos(theta_y)],
        ]
    )

    rot_z = np.array(
        [
            [math.cos(theta_z), -math.sin(theta_z), 0],
            [math.sin(theta_z), math.cos(theta_z), 0],
            [0, 0, 1],
        ]
    )

    rot_matrix = rot_z.dot(rot_y.dot(rot_x))
    rot_pointcloud = rot_matrix.dot(pointcloud.T).T
    return rot_pointcloud, rot_matrix


def point_line_distance(
    point: NDArray[np.float32],
    line_point: NDArray[np.float32],
    line_vector: NDArray[np.float32],
) -> float:
    """_summary_

    Args:
        point (NDArray[np.float32]): _description_
        line (np.ndarray(1, 6)): _description_

    Returns:
        float: _description_
    """
    u = point - line_point
    v = line_vector
    return abs(np.cross(u, v) / np.linalg.norm(u))
