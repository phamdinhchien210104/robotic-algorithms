"""
esdf_generator.py - Module trích xuất trường khoảng cách có dấu (ESDF).
Kế thừa giải thuật từ Mapping/DistanceMap/distance_map.py (dùng scipy.ndimage EDT tối ưu).
"""

import numpy as np
import scipy.ndimage


def compute_esdf_from_probability(prob_map, occ_threshold=0.65, resolution=0.1):
    """
    Nhận đầu vào là ma trận xác suất Occupancy Grid Map (2D).
    Trả về:
        esdf: Ma trận khoảng cách có dấu [mét]
              Dương (+) tại vùng an toàn (khoảng cách tới vật cản gần nhất).
              Âm (-) bên trong vật cản.
    """
    # 1: Vật cản (Occupied), 0: Vùng trống (Free)
    binary_obstacles = (prob_map >= occ_threshold).astype(np.int32)

    # distance_transform_edt tính cự ly đến điểm 0 gần nhất
    dist_to_obs = scipy.ndimage.distance_transform_edt(binary_obstacles == 0) * resolution
    dist_to_free = scipy.ndimage.distance_transform_edt(binary_obstacles == 1) * resolution

    # ESDF = khoảng cách ra vật cản - khoảng cách lún vào vật cản
    esdf = dist_to_obs - dist_to_free
    return esdf
