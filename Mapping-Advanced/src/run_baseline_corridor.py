"""
run_baseline_corridor.py - Chạy thuật toán Mapping gốc (Baseline) trên kịch bản hành lang.

Mục đích:
1. Cho robot di chuyển và người đi bộ cắt ngang hành lang (từ t = 3.0s đến 6.2s).
2. Dùng đúng thuật toán Log-Odds / Bresenham gốc của repo.
3. Xuất ra:
   - Ảnh chụp hiện tượng Ghost Tracks (vệt cản ảo kéo dài sau khi người đi bộ đã biến mất).
   - Biểu đồ và file CSV đo 3 chỉ số: Số ô cản ảo theo thời gian, RAM tiêu thụ, và Latency (ms/frame).
   - Lưu toàn bộ kết quả vào comparation/baseline/
"""

import os
import sys
import time
import math
import tracemalloc
import numpy as np
import matplotlib.pyplot as plt

# Thêm đường dẫn gốc vào sys.path để import đúng chuẩn
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Mapping.lidar_to_grid_map.lidar_to_grid_map import bresenham
from sim_corridor import SimulatedCorridor, VirtualLidar


class BaselineOccupancyGridMap:
    def __init__(self, min_x=-2.0, max_x=28.0, min_y=-2.0, max_y=6.0, resolution=0.1):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.resolution = resolution
        
        self.width = int(round((max_x - min_x) / resolution))
        self.height = int(round((max_y - min_y) / resolution))
        
        # Mảng 2D tĩnh toàn cục đúng chuẩn của thuật toán gốc trong repo
        # 0.5 tương ứng Log-Odds = 0 (trạng thái chưa biết / unknown)
        self.log_odds = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Các tham số cập nhật Bayes / Log-Odds gốc
        # P_occ = 0.7 (L_occ ~ 0.85), P_free = 0.4 (L_free ~ -0.40)
        self.l_occ = math.log(0.7 / (1.0 - 0.7))
        self.l_free = math.log(0.4 / (1.0 - 0.4))
        self.l_max = 5.0
        self.l_min = -5.0

    def world_to_grid(self, x, y):
        ix = int(round((x - self.min_x) / self.resolution))
        iy = int(round((y - self.min_y) / self.resolution))
        return ix, iy

    def update(self, rx, ry, hit_x, hit_y):
        """Cập nhật bản đồ bằng Bresenham chuẩn như trong Mapping/lidar_to_grid_map"""
        r_ix, r_iy = self.world_to_grid(rx, ry)
        
        for hx, hy in zip(hit_x, hit_y):
            h_ix, h_iy = self.world_to_grid(hx, hy)
            
            # Kiểm tra biên lưới
            if not (0 <= h_ix < self.width and 0 <= h_iy < self.height):
                continue

            # Sử dụng thuật toán Bresenham từ file gốc
            line = bresenham((r_ix, r_iy), (h_ix, h_iy))
            
            # Các ô trên tia (trừ ô cuối cùng) là Free space
            for px, py in line[:-1]:
                if 0 <= px < self.width and 0 <= py < self.height:
                    self.log_odds[px, py] = max(self.l_min, self.log_odds[px, py] + self.l_free)
                    
            # Ô cuối cùng tại điểm va chạm là Occupied space
            self.log_odds[h_ix, h_iy] = min(self.l_max, self.log_odds[h_ix, h_iy] + self.l_occ)

    def get_probability_map(self):
        """Chuyển log-odds sang xác suất P = 1 / (1 + exp(-L))"""
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def count_ghost_cells(self):
        """
        Đếm số ô bị đánh dấu Occupied (P > 0.65) ở lòng hành lang nơi người đi bộ đi qua
        Khu vực người đi bộ: x từ 9.0m đến 11.0m, y từ 0.6m đến 2.9m (tránh mép tường tĩnh).
        """
        prob = self.get_probability_map()
        ix1, iy1 = self.world_to_grid(9.2, 0.6)
        ix2, iy2 = self.world_to_grid(10.8, 2.9)
        
        region = prob[min(ix1, ix2):max(ix1, ix2), min(iy1, iy2):max(iy1, iy2)]
        return int(np.sum(region > 0.65))


def main():
    print("=== BẮT ĐẦU CHẠY THỰC NGHIỆM BASELINE MAPPING TRÊN HÀNH LANG BỆNH VIỆN ===")
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "comparation", "baseline"))
    os.makedirs(output_dir, exist_ok=True)

    corridor = SimulatedCorridor(length=25.0, width=3.5)
    lidar = VirtualLidar(num_rays=360, max_range=12.0)
    gmap = BaselineOccupancyGridMap(resolution=0.1)

    duration = 12.0  # Chạy 12 giây mô phỏng (tần số 10Hz = 120 frames)
    dt = 0.1
    steps = int(duration / dt)

    # Đo bộ nhớ heap bằng tracemalloc
    tracemalloc.start()

    time_records = []
    ghost_records = []
    latency_records = []
    ram_records = []

    # Lưu lại một số mốc ảnh quan trọng:
    # 1. Lúc người đang đi giữa hành lang (t = 4.5s)
    # 2. Ngay khi người vừa biến mất (t = 6.3s)
    # 3. Sau khi người biến mất khá lâu (t = 10.0s) để chứng minh vết ma không tự xóa
    snapshot_times = [4.5, 6.3, 10.0]
    snapshots = {}

    for step in range(steps):
        t = step * dt
        rx, ry, r_theta = corridor.get_robot_pose(t)
        
        # 1. LiDAR quét
        hit_x, hit_y, angles, ranges = lidar.scan(corridor, rx, ry, r_theta, t)
        
        # 2. Đo thời gian cập nhật bản đồ (Latency ms)
        t_start = time.perf_counter()
        gmap.update(rx, ry, hit_x, hit_y)
        t_cost_ms = (time.perf_counter() - t_start) * 1000.0
        
        # 3. Đo bộ nhớ RAM hiện tại
        current_ram_mb = tracemalloc.get_traced_memory()[0] / (1024 * 1024)
        
        # 4. Đếm số ô cản ảo trong hành lang
        ghost_count = gmap.count_ghost_cells()

        time_records.append(t)
        latency_records.append(t_cost_ms)
        ram_records.append(current_ram_mb)
        ghost_records.append(ghost_count)

        # Lưu ảnh snapshot
        for snap_t in snapshot_times:
            if abs(t - snap_t) < 0.05 and snap_t not in snapshots:
                snapshots[snap_t] = (gmap.get_probability_map().copy(), rx, ry, corridor.get_pedestrian_pos(t))

        if step % 20 == 0:
            print(f"Frame {step:3d}/{steps} | t={t:4.1f}s | Latency={t_cost_ms:5.1f}ms | Ghost Cells={ghost_count:2d} | RAM={current_ram_mb:4.2f}MB")

    tracemalloc.stop()

    # --- 1. XUẤT ẢNH MINH CHỨNG HIỆN TƯỢNG GHOST TRACKS ---
    print("\n[1/3] Đang xuất ảnh chụp hiện tượng Ghost Tracks...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), dpi=200)
    titles = [
        "t = 4.5s: Người đi bộ đang cắt ngang hành lang (Được phát hiện và ghi nhận)",
        "t = 6.3s: Người đi bộ ĐÃ RỜI ĐI vào phòng (Vết đen cản ảo bắt đầu tồn lưu)",
        "t = 10.0s: Người đã đi mất 3.7 giây trước - VỆT CẢN ẢO VẪN TỒN TẠI (GHOST TRACK)"
    ]

    for idx, snap_t in enumerate(snapshot_times):
        pmap, rx, ry, ped = snapshots[snap_t]
        ax = axes[idx]
        # Hiển thị xác suất: Trắng = Free, Đen = Occupied
        im = ax.imshow(1.0 - pmap.T, cmap="gray", origin="lower",
                       extent=[gmap.min_x, gmap.max_x, gmap.min_y, gmap.max_y], vmin=0, vmax=1)
        
        # Vẽ vị trí robot
        ax.plot(rx, ry, 'bo', markersize=8, label="Robot")
        # Vẽ vị trí thật của người (nếu có)
        if ped is not None:
            ax.plot(ped[0], ped[1], 'ro', markersize=9, label="Người thật (Ground Truth)")

        # Khoanh vùng khu vực có vệt bóng ma
        rect = plt.Rectangle((9.0, 0.5), 2.0, 2.5, fill=False, edgecolor='red', linewidth=1.8, linestyle='--')
        ax.add_patch(rect)
        ax.text(10.0, 3.1, "Vùng người cắt ngang", color='red', fontsize=9, fontweight='bold', ha='center')

        ax.set_title(titles[idx], fontsize=11, fontweight='bold', color='#1e293b')
        ax.set_xlim(-1.0, 26.0)
        ax.set_ylim(-1.0, 5.0)
        ax.set_ylabel("Y [m]")
        if idx == 2:
            ax.set_xlabel("X [m]")
        ax.legend(loc="upper left")

    plt.tight_layout()
    ghost_img_path = os.path.join(output_dir, "baseline_ghost_tracks.png")
    plt.savefig(ghost_img_path)
    plt.close()
    print(f" -> Đã lưu ảnh minh chứng vệt ma tại: {ghost_img_path}")

    # --- 2. XUẤT ĐỒ THỊ 3 CHỈ TIÊU ĐỊNH LƯỢNG ---
    print("[2/3] Đang xuất biểu đồ định lượng 3 chỉ tiêu...")
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), dpi=200, sharex=True)

    # 1. Số ô cản ảo
    ax1.plot(time_records, ghost_records, 'r-', linewidth=2, label="Ghost cells (Baseline)")
    ax1.axvspan(3.0, 6.2, color='orange', alpha=0.25, label="Thời gian người thật xuất hiện")
    ax1.set_ylabel("Số ô cản ảo (Cells)")
    ax1.set_title("Chỉ tiêu 1: Số ô cản ảo tồn lưu theo thời gian (Không tự triệt tiêu)", fontweight='bold')
    ax1.grid(True, linestyle=":")
    ax1.legend(loc="upper right")

    # 2. RAM chiếm dụng
    ax2.plot(time_records, ram_records, 'b-', linewidth=1.8)
    ax2.set_ylabel("RAM Heap [MB]")
    ax2.set_title("Chỉ tiêu 2: Dung lượng RAM tiêu thụ (Ma trận lưới 2D tĩnh)", fontweight='bold')
    ax2.grid(True, linestyle=":")

    # 3. Thời gian tính toán mỗi frame
    ax3.plot(time_records, latency_records, 'g-', linewidth=1.2, alpha=0.8)
    ax3.axhline(50.0, color='red', linestyle='--', label="Ngưỡng thời gian thực 50ms (20Hz)")
    ax3.set_xlabel("Thời gian mô phỏng [giây]")
    ax3.set_ylabel("Độ trễ [ms/frame]")
    ax3.set_title(f"Chỉ tiêu 3: Thời gian tính toán trung bình: {np.mean(latency_records):.2f} ms/frame", fontweight='bold')
    ax3.grid(True, linestyle=":")
    ax3.legend(loc="upper right")

    plt.tight_layout()
    metric_img_path = os.path.join(output_dir, "baseline_quantitative_metrics.png")
    plt.savefig(metric_img_path)
    plt.close()
    print(f" -> Đã lưu biểu đồ định lượng tại: {metric_img_path}")

    # --- 3. XUẤT FILE SỐ LIỆU CSV LÀM BẰNG CHỨNG ---
    csv_path = os.path.join(output_dir, "baseline_metrics.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("time_s,ghost_cells,latency_ms,ram_mb\n")
        for t, g, l, r in zip(time_records, ghost_records, latency_records, ram_records):
            f.write(f"{t:.2f},{g},{l:.3f},{r:.3f}\n")
    print(f"[3/3] Đã xuất file CSV số liệu tại: {csv_path}")

    print("\n=== HOÀN THÀNH BƯỚC 1 & 2 CHO BẢN BASELINE ===")


if __name__ == "__main__":
    main()
