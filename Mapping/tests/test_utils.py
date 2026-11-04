import conftest  # noqa
from utils import angle
import numpy as np


def test_angle():
    rot = angle.rot_mat_2d(0.0)
    assert np.allclose(rot, np.eye(2))
