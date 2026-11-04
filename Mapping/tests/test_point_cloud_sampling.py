import conftest  # noqa
from Mapping.point_cloud_sampling import point_cloud_sampling as m


def test_voxel_point_sampling():
    m.do_plot = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
