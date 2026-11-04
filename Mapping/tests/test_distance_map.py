import conftest  # noqa
from Mapping.DistanceMap import distance_map as m
import numpy as np


def test_sdf():
    grid = np.array([
        [0, 1, 0],
        [1, 1, 1],
        [0, 1, 0]
    ])
    sdf = m.compute_sdf_scipy(grid)
    assert sdf is not None
    assert sdf.shape == grid.shape


def test_udf():
    grid = np.array([
        [0, 1, 0],
        [1, 1, 1],
        [0, 1, 0]
    ])
    udf = m.compute_udf_scipy(grid)
    assert udf is not None
    assert udf.shape == grid.shape
