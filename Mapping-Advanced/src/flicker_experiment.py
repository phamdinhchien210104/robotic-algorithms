"""
flicker_experiment.py - Thực nghiệm chứng minh hiện tượng giật cục / nhấp nháy (Flickering).

Kịch bản:
- Một người đi bộ đang đứng yên tại x=10m phía trước robot.
- Ở các giây t = 4.0s -> 4.5s (trong 0.5 giây), cảm biến LiDAR bị 'hụt tia' (sensor dropout) do góc quét hoặc nhiễu tia phản xạ.
- Ngay sau đó t = 4.6s, cảm biến lại bắt được tia va chạm bình thường.
- Ta đo:
  1. Xác suất ô chiếm dụng P(m) theo thời gian.
  2. Khoảng cách an toàn ESDF tại vị trí robot: nếu ESDF nhảy vọt rồi tụt dốc đột ngột -> robot sẽ bị giật cục (đang phanh bỗng nhấn ga rồi lại phanh gấp).
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from grid_map_core import TimeDecayGridMap, EvaporationGridMap
from esdf_generator import compute_esdf_from_probability

def run_flicker_test():
    gmap_time = TimeDecayGridMap(resolution=0.1, decay_rate=0.55)
    gmap_evap = EvaporationGridMap(resolution=0.1, alpha=0.25, window_size=4) # window 4 frames

    time_steps = np.arange(0.0, 7.0, 0.1)
    
    p_time_history = []
    p_evap_history = []
    esdf_time_history = []
    esdf_evap_history = []

    ix, iy = gmap_time.world_to_grid(10.0, 1.75)
    r_ix, r_iy = gmap_time.world_to_grid(5.0, 1.75)

    for t in time_steps:
        # Bình thường luôn có tia va chạm vào người ở (10.0, 1.75)
        # Nhưng từ t = 3.5s đến 4.0s: bị hụt tia (dropout)
        if 3.5 <= t <= 4.0:
            hits_x, hits_y = [], []
        else:
            hits_x, hits_y = [10.0], [1.75]

        gmap_time.update(rx=5.0, ry=1.75, hit_x=hits_x, hit_y=hits_y, dt=0.1)
        gmap_evap.update(rx=5.0, ry=1.75, hit_x=hits_x, hit_y=hits_y, dt=0.1)

        pt = gmap_time.get_probability_map()[ix, iy]
        pe = gmap_evap.get_probability_map()[ix, iy]
        p_time_history.append(pt)
        p_evap_history.append(pe)

        # Tính ESDF
        esdf_t = compute_esdf_from_probability(gmap_time.get_probability_map(), resolution=0.1)[r_ix, r_iy]
        esdf_e = compute_esdf_from_probability(gmap_evap.get_probability_map(), resolution=0.1)[r_ix, r_iy]
        esdf_time_history.append(esdf_t)
        esdf_evap_history.append(esdf_e)

    # Vẽ biểu đồ chứng minh hiện tượng giật cục
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), dpi=200, sharex=True)

    ax1.plot(time_steps, p_time_history, 'b-', linewidth=2.0, label="Time-decay: P(m) suy giảm mượt")
    ax1.plot(time_steps, p_evap_history, 'r--', linewidth=2.0, label="Evaporation: P(m) rơi tự do (Nhấp nháy)")
    ax1.axvspan(3.5, 4.0, color='orange', alpha=0.25, label="Khoảng thời gian LiDAR bị nhiễu / hụt tia (0.5s)")
    ax1.axhline(0.65, color='gray', linestyle=':', label="Ngưỡng phân loại vật cản (Threshold = 0.65)")
    ax1.set_ylabel("Xác suất chiếm dụng P")
    ax1.set_title("HIỆN TƯỢNG NHẤP NHÁY (MAP FLICKERING) KHI CẢM BIẾN BỊ HỤT TIA TẠM THỜI", fontweight='bold')
    ax1.grid(True, linestyle=":")
    ax1.legend(loc="lower left")

    # Đồ thị ESDF (ảnh hưởng tới điều khiển vận tốc robot)
    ax2.plot(time_steps, esdf_time_history, 'b-', linewidth=2.0, label="Time-decay: Cự ly an toàn giữ vững ổn định")
    ax2.plot(time_steps, esdf_evap_history, 'r--', linewidth=2.0, label="Evaporation: Cự ly an toàn nhảy vọt rồi tụt dốc (GIẬT CỤC)")
    ax2.axvspan(3.5, 4.0, color='orange', alpha=0.25)
    ax2.set_xlabel("Thời gian mô phỏng [giây]")
    ax2.set_ylabel("Cự ly an toàn ESDF [m]")
    ax2.set_title("ẢNH HƯỞNG TỚI ĐIỀU KHIỂN: Robot bị giật cục (đang phanh -> tưởng đường thông đạp ga -> phanh gấp)", fontweight='bold')
    ax2.grid(True, linestyle=":")
    ax2.legend(loc="lower left")

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "comparation", "flickering_analysis.png")
    plt.savefig(out_path)
    plt.close()
    print("Xuat thanh cong anh phan tich giat cuc tai:", out_path)

if __name__ == "__main__":
    run_flicker_test()
