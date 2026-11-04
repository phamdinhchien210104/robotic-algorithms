"""
plot_language_comparison.py - Tổng hợp và vẽ biểu đồ đối sánh đa ngôn ngữ:
Python vs C++ vs Rust
Lưu đồ thị và file CSV tổng hợp vào comparation/languages/
"""

import os
import matplotlib.pyplot as plt
import numpy as np

output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "comparation", "languages"))
os.makedirs(output_dir, exist_ok=True)

# 1. Số liệu thực nghiệm đo đạc chính xác từ 3 ngôn ngữ:
languages = ["Python (NumPy)", "C++ (g++ -O3)", "Rust (cargo --release)"]

# Baseline Latency (ms/frame)
base_lat = [17.26, 0.032, 0.155]
# Cải tiến Time-decay Sparse Latency (ms/frame)
sparse_lat = [13.72, 0.143, 0.213]

# RAM Heap (MB)
dense_ram = [0.99, 0.99, 0.99]
sparse_ram = [0.14, 0.13, 0.13]

# --- VẼ ĐỒ THỊ SO SÁNH 3 NGÔN NGỮ ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=200)

# Đồ thị 1: So sánh Tốc độ Latency (Log Scale để thấy rõ cả Python lẫn C++/Rust)
x = np.arange(len(languages))
width = 0.35

rects1 = ax1.bar(x - width/2, base_lat, width, label='Baseline (Gốc)', color='#ef4444')
rects2 = ax1.bar(x + width/2, sparse_lat, width, label='Cải tiến (Sparse Time-decay)', color='#3b82f6')

ax1.set_ylabel('Độ trễ mỗi frame [ms/frame] (Thang Logarithm)')
ax1.set_title('SO SÁNH TỐC ĐỘ XỬ LÝ (LATENCY) THEO NGÔN NGỮ', fontweight='bold')
ax1.set_xticks(x)
ax1.set_xticklabels(languages, fontweight='bold')
ax1.set_yscale('log')
ax1.axhline(50.0, color='black', linestyle=':', label='Ngưỡng Real-time 50ms')
ax1.legend()
ax1.grid(True, linestyle=":", alpha=0.6)

for r in rects1:
    h = r.get_height()
    ax1.annotate(f'{h:.3f} ms', xy=(r.get_x() + r.get_width() / 2, h),
                xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')

for r in rects2:
    h = r.get_height()
    ax1.annotate(f'{h:.3f} ms', xy=(r.get_x() + r.get_width() / 2, h),
                xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')

# Đồ thị 2: So sánh Bộ nhớ RAM (Hành lang 60m)
rects3 = ax2.bar(x - width/2, dense_ram, width, label='Dense Grid (Mảng tĩnh)', color='#ef4444')
rects4 = ax2.bar(x + width/2, sparse_ram, width, label='Chunk Sparse (Mảng động)', color='#10b981')

ax2.set_ylabel('Dung lượng RAM [MB]')
ax2.set_title('SO SÁNH BỘ NHỚ RAM TIÊU THỤ THEO NGÔN NGỮ', fontweight='bold')
ax2.set_xticks(x)
ax2.set_xticklabels(languages, fontweight='bold')
ax2.set_ylim(0, 1.3)
ax2.legend()
ax2.grid(True, linestyle=":", alpha=0.6)

for r in rects3:
    h = r.get_height()
    ax2.annotate(f'{h:.2f} MB', xy=(r.get_x() + r.get_width() / 2, h),
                xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')

for r in rects4:
    h = r.get_height()
    ax2.annotate(f'{h:.2f} MB', xy=(r.get_x() + r.get_width() / 2, h),
                xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')

plt.tight_layout()
chart_path = os.path.join(output_dir, "languages_comparison_benchmark.png")
plt.savefig(chart_path)
plt.close()

# Xuất CSV đối sánh đa ngôn ngữ
csv_path = os.path.join(output_dir, "languages_summary.csv")
with open(csv_path, "w", encoding="utf-8") as f:
    f.write("language,baseline_latency_ms,sparse_timedecay_latency_ms,dense_ram_mb,sparse_ram_mb,ghost_removal_time\n")
    f.write(f"Python (Pure),{base_lat[0]:.2f},{sparse_lat[0]:.2f},{dense_ram[0]:.2f},{sparse_ram[0]:.2f},0.8s\n")
    f.write(f"C++ (g++ -O3),{base_lat[1]:.4f},{sparse_lat[1]:.4f},{dense_ram[1]:.2f},{sparse_ram[1]:.2f},0.8s\n")
    f.write(f"Rust (Release),{base_lat[2]:.4f},{sparse_lat[2]:.4f},{dense_ram[2]:.2f},{sparse_ram[2]:.2f},0.8s\n")

print(f"Da xuat thanh cong bieu do va CSV vao: {output_dir}")
