import numpy as np
from ply_processor.geometry import point_line_distance


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
