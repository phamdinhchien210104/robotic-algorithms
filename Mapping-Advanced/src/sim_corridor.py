"""
sim_corridor.py - Bộ giả lập đa kịch bản hành lang bệnh viện phục vụ kiểm chứng định lượng.

Tuân thủ nghiêm ngặt theo Định hướng.md và SPEC:
- Kịch bản 1 ('crossing'): Người đi bộ cắt ngang hành lang (t = 3.0s -> 6.2s) rồi biến mất vào phòng đối diện.
- Kịch bản 2 ('oncoming'): Người đi ngược chiều đối diện robot (t = 3.0s -> 7.0s) rồi rẽ vào phòng khám.
- Kịch bản 3 ('long_corridor'): Hành lang dài 60m để so sánh RAM giữa mảng tĩnh và lưới thưa (Sparse Grid).
"""

import math
import numpy as np


class SimulatedCorridor:
    def __init__(self, scenario="crossing", length=25.0, width=3.5):
        self.scenario = scenario
        self.length = 60.0 if scenario == "long_corridor" else length
        self.width = width
        
        # Định nghĩa 4 bức tường hành lang tĩnh
        self.static_walls = [
            (0.0, 0.0, self.length, 0.0),            # Tường dưới (y = 0)
            (0.0, self.width, self.length, self.width), # Tường trên (y = width)
            (0.0, 0.0, 0.0, self.width),             # Tường đầu hành lang
            (self.length, 0.0, self.length, self.width), # Tường cuối hành lang
        ]

    def get_robot_pose(self, t):
        """
        Vị trí robot (rx, ry, theta) tại thời điểm t.
        Vận tốc v = 0.5 m/s chạy thẳng dọc tim hành lang.
        """
        rx = 2.0 + 0.5 * t
        rx = min(rx, self.length - 2.0)
        ry = self.width / 2.0
        theta = 0.0
        return rx, ry, theta

    def get_pedestrian_pos(self, t):
        """
        Trả về tọa độ (px, py, radius) của người đi bộ.
        Trả về None nếu người chưa xuất hiện hoặc đã đi vào phòng.
        """
        if self.scenario == "crossing":
            # Kịch bản 1: Cắt ngang từ y=0.2m sang y=3.3m tại x=10m trong 3.2s
            if 3.0 <= t <= 6.2:
                dt = t - 3.0
                py = 0.2 + 1.0 * dt
                px = 10.0 + 0.05 * math.sin(dt * 2.0)
                return (px, py, 0.3)
            return None

        elif self.scenario == "oncoming":
            # Kịch bản 2: Đi ngược chiều đối diện robot từ x=16m về x=8m, sau đó rẽ vào phòng lúc t=6.5s
            if 2.5 <= t <= 6.5:
                dt = t - 2.5
                px = 16.0 - 1.2 * dt  # Đi ngược về phía robot
                py = 2.4              # Đi ở làn trên của hành lang
                return (px, py, 0.3)
            elif 6.5 < t <= 7.3:
                # Rẽ vào phòng ở tường trên y = 3.5
                dt = t - 6.5
                px = 16.0 - 1.2 * 4.0
                py = 2.4 + 1.2 * dt
                return (px, py, 0.3)
            return None

        elif self.scenario == "long_corridor":
            # Kịch bản 3: Hành lang dài, có 2 người xuất hiện ở các vị trí khác nhau
            # Người 1: x = 15m (t = 5s -> 8s)
            if 5.0 <= t <= 8.2:
                dt = t - 5.0
                return (15.0, 0.3 + 0.9 * dt, 0.3)
            # Người 2: x = 35m (t = 15s -> 18s)
            if 15.0 <= t <= 18.2:
                dt = t - 15.0
                return (35.0, 3.2 - 0.9 * dt, 0.3)
            return None

        return None


class VirtualLidar:
    def __init__(self, num_rays=360, max_range=12.0):
        self.num_rays = num_rays
        self.max_range = max_range
        self.angles = np.linspace(-math.pi, math.pi, num_rays, endpoint=False)

    def scan(self, corridor: SimulatedCorridor, rx, ry, r_theta, t):
        """
        Bắn tia 2D tìm giao điểm gần nhất với tường tĩnh và người đi bộ.
        """
        pedestrian = corridor.get_pedestrian_pos(t)
        hit_x = []
        hit_y = []
        ranges = []

        for angle in self.angles:
            global_angle = r_theta + angle
            cos_a = math.cos(global_angle)
            sin_a = math.sin(global_angle)

            min_dist = self.max_range

            # 1. Kiểm tra va chạm với 4 bức tường thẳng
            for (x1, y1, x2, y2) in corridor.static_walls:
                d = self._ray_segment_intersect(rx, ry, cos_a, sin_a, x1, y1, x2, y2)
                if d is not None and 0.0 < d < min_dist:
                    min_dist = d

            # 2. Kiểm tra va chạm với người đi bộ (hình trụ tròn)
            if pedestrian is not None:
                px, py, pr = pedestrian
                d_ped = self._ray_circle_intersect(rx, ry, cos_a, sin_a, px, py, pr)
                if d_ped is not None and 0.0 < d_ped < min_dist:
                    min_dist = d_ped

            # Ghi nhận điểm va chạm trong tầm quét
            if min_dist < self.max_range:
                # Thêm nhiễu Gaussian cảm biến thực tế (sigma = 0.01m)
                noise = np.random.normal(0, 0.01)
                d_noisy = max(0.1, min_dist + noise)
                hit_x.append(rx + d_noisy * cos_a)
                hit_y.append(ry + d_noisy * sin_a)
                ranges.append(d_noisy)
            else:
                ranges.append(self.max_range)

        return np.array(hit_x), np.array(hit_y), self.angles, np.array(ranges)

    @staticmethod
    def _ray_segment_intersect(rx, ry, cos_a, sin_a, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        denom = cos_a * dy - sin_a * dx
        if abs(denom) < 1e-6:
            return None
        t = ((x1 - rx) * dy - (y1 - ry) * dx) / denom
        u = ((x1 - rx) * sin_a - (y1 - ry) * cos_a) / denom
        if t > 0.05 and 0.0 <= u <= 1.0:
            return t
        return None

    @staticmethod
    def _ray_circle_intersect(rx, ry, cos_a, sin_a, cx, cy, r):
        fx = rx - cx
        fy = ry - cy
        b = 2.0 * (fx * cos_a + fy * sin_a)
        c = fx * fx + fy * fy - r * r
        discriminant = b * b - 4.0 * c
        if discriminant >= 0:
            sqrt_d = math.sqrt(discriminant)
            t1 = (-b - sqrt_d) / 2.0
            t2 = (-b + sqrt_d) / 2.0
            if t1 > 0.05:
                return t1
            if t2 > 0.05:
                return t2
        return None
