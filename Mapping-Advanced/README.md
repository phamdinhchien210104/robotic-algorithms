# Mapping-Advanced: Cải tiến Thuật toán Ray-Casting Occupancy Grid Map

Dự án cải tiến thuật toán xây dựng bản đồ lưới cho robot phục vụ trong môi trường động hành lang bệnh viện đông người, theo đặc tả kỹ thuật trong bản SPEC `K.T.Robot & Chuyên đề 2 - CK`.

## Cấu trúc thư mục

```text
Mapping-Advanced/
├── src/
│   ├── __init__.py
│   ├── sim_corridor.py         # Giả lập môi trường hành lang 20-30m x 3-4m & người đi bộ di động
│   ├── grid_map_core.py        # Các giải thuật Mapping: Baseline, Time-decay Log-Odds, Evaporation + Sparse Grid
│   ├── esdf_generator.py       # Tích hợp trích xuất trường khoảng cách có dấu (ESDF/EDT)
│   └── evaluate_benchmark.py   # Benchmark định lượng 3 chỉ số (Ghost track, RAM, Thời gian tính toán)
├── tests/
│   ├── __init__.py
│   └── test_advanced_mapping.py# Bộ kiểm thử đơn vị & kiểm thử tích hợp
├── results/                    # Nơi lưu trữ biểu đồ, hình ảnh xuất ra và file số liệu benchmark
├── __init__.py
└── README.md
```

## Các lệnh thực thi chính (Commands)

1. **Chạy mô phỏng trực quan so sánh 3 phương pháp:**
   ```bash
   python Mapping-Advanced/src/sim_corridor.py --mode comparison --duration 20
   ```

2. **Chạy bộ thử nghiệm và xuất kết quả định lượng phục vụ báo cáo:**
   ```bash
   python Mapping-Advanced/src/evaluate_benchmark.py --export-plots --output-dir Mapping-Advanced/results/
   ```
