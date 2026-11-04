import conftest  # noqa
from Mapping.grid_map_lib import grid_map_lib as m
import matplotlib.pyplot as plt


def test1():
    plt.close('all')
    m.position_set_demo()
    m.polygon_set_demo()
    plt.close('all')


if __name__ == '__main__':
    conftest.run_this_test(__file__)
