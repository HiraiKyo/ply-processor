from ply_processor.cylinder.fixed_axis import find_circle
import numpy as np
import pytest
import math


def test_find_circle():
    a0 = np.array([1, 1, 0])
    b0 = np.array([2, 1, 0])
    c0 = np.array([1, 2, 0])
    assert find_circle(a0, b0, c0) == (
        pytest.approx(1.5),
        pytest.approx(1.5),
        pytest.approx(1 / math.sqrt(2)),
    )

    a1 = np.array([1, 1.5, 0])
    b1 = np.array([10.5, 1.5, 0])
    c1 = np.array([1, 20, 0])
    est_r = math.sqrt((b1[0] - c1[0]) ** 2 + (b1[1] - c1[1]) ** 2) / 2
    assert find_circle(a1, b1, c1)[2] == pytest.approx(est_r)
