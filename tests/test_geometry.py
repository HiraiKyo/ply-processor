import numpy as np
from ply_processor.geometry import point_line_distance, get_rotation_matrix_from_vectors
import numpy.testing as npt


# point_line_distanceのテスト
def test_point_line_distance():
    point = np.array([1, 1, 1])
    line_point = np.array([0, 0, 0])
    line_vector = np.array([1, 0, 0])
    assert (
        point_line_distance(point, line_point, line_vector) <= 1.415
        and point_line_distance(point, line_point, line_vector) >= 1.414
    )


def test_point_line_distance_2():
    point = np.array([[1, 1, 1], [1, 2, 2]])
    line_point = np.array([0, 0, 0])
    line_vector = np.array([1, 0, 0])
    distances = point_line_distance(point, line_point, line_vector)
    assert distances[0] <= 1.415 and distances[0] >= 1.414
    assert distances[1] <= 2.829 and distances[1] >= 2.828


def test_get_rotation_matrix_from_vectors():
    v0 = np.array([0, 0, 1])
    v0o = np.array([1, 0, 0])
    rot_matrix = get_rotation_matrix_from_vectors(v0, v0o)
    npt.assert_allclose(
        rot_matrix, np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]), atol=1e-10
    )
