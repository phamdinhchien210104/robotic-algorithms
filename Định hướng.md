# Nghiên cứu và cải tiến thuật toán Ray-casting Occupancy Grid Map cho robot phục vụ trong môi trường động hành lang bệnh viện
Robot phục vụ trong hành lang bệnh viện: Nơi mật độ người đi bộ cao, đòi hỏi xóa vết bóng người chuyển động theo thời gian thực.

Về nguyên lý, thuật toán chuyển đổi dữ liệu khoảng cách từ cảm biến LiDAR thành xác suất chiếm dụng của từng ô lưới 2D trong không gian. Tuy nhiên, phương pháp truyền thống bộc lộ ba điểm yếu cốt lõi: thứ nhất, giả định môi trường tĩnh tuyệt đối dẫn đến hiện tượng lưu lại các vệt vật cản ảo kéo dài (ghost tracks) khi có người hoặc phương tiện di chuyển qua; thứ hai, cấu trúc mảng lưới 2D tĩnh tiêu tốn tài nguyên bộ nhớ lớn theo diện tích và bỏ qua hoàn toàn thông tin cao độ địa hình; thứ ba, tốc độ tính toán tia tuần tự chưa đáp ứng tốt yêu cầu xử lý thời gian thực ở các cảm biến mật độ cao. Để khắc phục các hạn chế này, dự án tập trung vào mục tiêu trọng tâm là cải tiến cơ chế cập nhật xác suất có suy giảm theo thời gian (Temporal Evaporation / Dynamic Filter) nhằm tự động xóa sạch bóng ma vật cản động theo thời gian thực; đồng thời xem xét tích hợp cấu trúc lưới thưa và trường khoảng cách có dấu (ESDF) làm hướng mở rộng bổ trợ cho việc tối ưu bộ nhớ và lập kế hoạch di chuyển an toàn cho robot.

## Chỉ tiêu kiểm chứng định lượng
- Thời gian xóa vết vật cản ảo: Đo số chu kỳ quét (hoặc số giây) để một bóng người đi qua biến mất khỏi bản đồ sau khi áp dụng cơ chế suy giảm xác suất so với thuật toán gốc.
- Bộ nhớ chiếm dụng: So sánh dung lượng RAM giữa mảng lưới tĩnh thông thường và cấu trúc lưới thưa khi robot di chuyển dọc hành lang dài.
- Thời gian tính toán mỗi frame: Đo độ trễ xử lý (ms/frame) trên dữ liệu lidar01.csv để chứng minh khả năng đáp ứng thời gian thực.

## Các thuật toán trong repo

### 1. `ray_casting_grid_map` (Bản đồ lưới dùng thuật toán Ray-Casting)
* **File mã nguồn:** [`ray_casting_grid_map.py`](file:///d:/Document/Robotic-Algorithms/Mapping/ray_casting_grid_map/ray_casting_grid_map.py)
* **Nguyên lý hoạt động:**
  * Giả lập robot phát ra các tia quét dạng quạt tròn xung quanh với góc từ $0 \rightarrow 2\pi$.
  * Tác giả sử dụng một bảng tra trước (`precastDB`) lưu trữ danh sách các điểm lưới theo góc và khoảng cách để tăng tốc việc kiểm tra tia.
  * Khi tia quét gặp điểm va chạm vật cản (obstacle):
    * Các ô nằm dọc theo đường truyền từ tâm robot tới trước vật cản được đánh dấu là **vùng trống (Free space)**.
    * Ô nằm ngay tại vị trí va chạm được đánh dấu là **vật cản (Occupied space)**.
    * Vùng phía sau vật thể bị che khuất được gán nhãn **chưa khám phá (Unknown)**.
* **Mục đích:** Xây dựng bản đồ lưới nhị phân/chiếm dụng từ dữ liệu khoảng cách laser quét 360 độ.

---

### 2. `lidar_to_grid_map` (Chuyển đổi dữ liệu LiDAR thành Occupancy Grid Map)
* **File mã nguồn:** [`lidar_to_grid_map.py`](file:///d:/Document/Robotic-Algorithms/Mapping/lidar_to_grid_map/lidar_to_grid_map.py) (đi kèm file log thực tế [`lidar01.csv`](file:///d:/Document/Robotic-Algorithms/Mapping/lidar_to_grid_map/lidar01.csv))
* **Nguyên lý hoạt động:**
  * Đọc dữ liệu góc quét (angle) và cự ly (distance) từ file CSV thực tế.
  * Sử dụng thuật toán vẽ đường thẳng số nguyên kinh điển **Bresenham’s Line Algorithm** để tìm nhanh các tọa độ ô lưới nằm trên đường truyền của tia laser từ robot tới vật cản.
  * Cập nhật xác suất theo mô hình Bayesian / Log-Odds:
    * Ô có tia xuyên qua: giảm xác suất bị chiếm dụng (`P_free`).
    * Ô tận cùng (nơi laser phản xạ lại): tăng xác suất bị chiếm dụng (`P_occ`).
* **Mục đích:** Mô phỏng quá trình tạo bản đồ chuẩn của xe tự hành từ cảm biến LiDAR 2D thực tế.

---

### 3. `gaussian_grid_map` (Bản đồ lưới phân bố Gauss / Đo độ không chắc chắn)
* **File mã nguồn:** [`gaussian_grid_map.py`](file:///d:/Document/Robotic-Algorithms/Mapping/gaussian_grid_map/gaussian_grid_map.py)
* **Nguyên lý hoạt động:**
  * Thay vì đánh dấu ô lưới cứng là 0 hoặc 1, thuật toán này tính toán khoảng cách ngắn nhất từ tâm của từng ô lưới tới các điểm vật cản.
  * Sử dụng hàm phân phối tích lũy Gauss (Cumulative Distribution Function - CDF từ `scipy.stats.norm`) với độ lệch chuẩn $\sigma$ (`std`):
    $$\text{pdf} = 1.0 - \text{norm.cdf}(\text{mindis}, 0.0, \sigma)$$
  * Các ô càng gần vật cản thì giá trị xác suất tiến dần về 1; càng xa vật cản thì xác suất giảm mượt dần về 0.
* **Mục đích:** Tạo ra bản đồ có xét đến sai số đo đạc của cảm biến (Sensor Noise / Measurement Uncertainty), thường dùng làm tiền đề cho Likelihood Field Model trong định vị robot.

---

### 4. `kmeans_clustering` (Phân cụm điểm vật thể bằng K-Means)
* **File mã nguồn:** [`kmeans_clustering.py`](file:///d:/Document/Robotic-Algorithms/Mapping/kmeans_clustering/kmeans_clustering.py)
* **Nguyên lý hoạt động:**
  * Khi LiDAR quét về một đám mây điểm (Point Cloud), các điểm này rời rạc và chưa biết điểm nào thuộc về cùng một vật thể.
  * Thuật toán K-Means gom các điểm laser có tọa độ gần nhau vào cùng một cụm (cluster) dựa trên khoảng cách Euclidean.
* **Mục đích:** Tiền xử lý dữ liệu trước khi nhận dạng vật thể (tách riêng mây điểm của xe A, xe B, người đi bộ).

---

### 5. `rectangle_fitting` (Khớp khung hình hộp chữ nhật / L-Shape Fitting)
* **File mã nguồn:** [`rectangle_fitting.py`](file:///d:/Document/Robotic-Algorithms/Mapping/rectangle_fitting/rectangle_fitting.py)
* **Nguyên lý hoạt động:**
  * Dựa trên nghiên cứu nổi tiếng của CMU (*"Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners"*).
  * Trong thực tế, cảm biến LiDAR gắn trên xe chỉ có thể quét thấy 1 hoặc 2 cạnh bên của các xe khác (tạo thành hình chữ "L").
  * Thuật toán xoay hệ trục tọa độ theo các góc quét $\theta \in [0, \pi/2]$ và tìm góc xoay tối ưu dựa trên 3 tiêu chí lựa chọn:
    1. **Area Criteria:** Diện tích bao quanh hình chữ nhật là nhỏ nhất.
    2. **Closeness Criteria:** Khoảng cách từ các điểm tới 2 cạnh chữ L là gần nhất.
    3. **Variance Criteria:** Phương sai phân bố điểm dọc theo các cạnh là nhỏ nhất.
* **Mục đích:** Xác định chính xác kích thước bao (Bounding Box: chiều dài, chiều rộng) và hướng quay (Heading angle / Orientation) của các phương tiện giao thông xung quanh.

### 6. `circle_fitting` (Khớp hình tròn bằng phương pháp bình phương tối thiểu)
* **File mã nguồn:** [`circle_fitting.py`](file:///d:/Document/Robotic-Algorithms/Mapping/circle_fitting/circle_fitting.py)
* **Nguyên lý hoạt động:**
  * Thuật toán giải bài toán tối ưu hình học: tìm tâm $(x_c, y_c)$ và bán kính $r$ sao cho tổng bình phương khoảng cách sai lệch từ các điểm quét $(x_i, y_i)$ đến đường tròn là nhỏ nhất.
  * Phương trình đường tròn được tuyến tính hóa về dạng ma trận đại số tuyến tính:
    $$2x_i x_c + 2y_i y_c + (r^2 - x_c^2 - y_c^2) = x_i^2 + y_i^2$$
  * Code tính toán các đại lượng tổng: $\sum x_i, \sum y_i, \sum x_i^2, \sum y_i^2, \sum x_i y_i, \dots$, từ đó thiết lập hệ phương trình chuẩn $3 \times 3$ và giải tìm trực tiếp nghiệm đóng (Closed-form solution) mà không cần lặp gradient.
  * Tính toán sai số trung bình (Fitting error) để xác định mức độ khớp của đám mây điểm.
* **Mục đích:** 
  * Nhận dạng và ước lượng vị trí, kích thước của các vật thể hình trụ tròn xung quanh robot như: cột đèn, thân cây, thùng phuy, chân bàn ghế hoặc các biển báo giao thông hình trụ.

---

### 7. `normal_vector_estimation` (Ước lượng Vector Pháp tuyến bề mặt 3D)
* **File mã nguồn:** [`normal_vector_estimation.py`](file:///d:/Document/Robotic-Algorithms/Mapping/normal_vector_estimation/normal_vector_estimation.py)
* **Nguyên lý hoạt động:**
  * **Trường hợp cơ bản (Tam giác 3 điểm):** Tính tích có hướng (Cross product) giữa hai vector cạnh $\vec{v}_1 = p_2 - p_1$ và $\vec{v}_2 = p_3 - p_1$, sau đó chuẩn hóa vector để thu được vector pháp tuyến đơn vị $\vec{n} = \frac{\vec{v}_1 \times \vec{v}_2}{\|\vec{v}_1 \times \vec{v}_2\|}$.
  * **Trường hợp tập điểm (Point Cloud PCA):** Với một cụm điểm lân cận, thuật toán tính ma trận hiệp phương sai (Covariance Matrix) của cụm điểm:
    $$C = \frac{1}{k} \sum_{i=1}^k (p_i - \bar{p})(p_i - \bar{p})^T$$
  * Phân tích trị riêng và vector riêng (Eigenvalue Decomposition). Vector riêng ứng với trị riêng nhỏ nhất chính là **vector pháp tuyến** của mặt phẳng chứa cụm điểm đó.
* **Mục đích:**
  * Giúp robot hiểu hình học môi trường 3D: phân biệt mặt đất bằng phẳng (pháp tuyến hướng thẳng đứng $[0, 0, 1]$), mặt dốc, tường đứng hoặc vật cản gồ ghề không thể đi qua (Traversability Analysis).

---

### 8. `point_cloud_sampling` (Lấy mẫu và giảm mật độ mây điểm - Downsampling)
* **File mã nguồn:** [`point_cloud_sampling.py`](file:///d:/Document/Robotic-Algorithms/Mapping/point_cloud_sampling/point_cloud_sampling.py)
* **Nguyên lý hoạt động:** Cung cấp 3 chiến lược lấy mẫu điểm khác nhau:
  1. **Voxel Point Sampling (Lấy mẫu theo lưới Voxel):** Chia không gian $N$ chiều thành các ô lập phương nhỏ có kích thước `voxel_size`. Tất cả các điểm rơi vào cùng một ô voxel sẽ được gom lại và lấy giá trị trung bình (Centroid/Mean) đại diện cho ô đó.
  2. **Farthest Point Sampling - FPS (Lấy mẫu điểm xa nhất):** Chọn điểm đầu tiên ngẫu nhiên, sau đó mỗi điểm tiếp theo được chọn phải có khoảng cách lớn nhất đến tập hợp các điểm đã chọn trước đó. Đảm bảo độ bao phủ hình học tối đa.
  3. **Poisson Disk Sampling:** Lấy mẫu ngẫu nhiên có kiểm soát sao cho không có 2 điểm nào nằm gần nhau hơn một bán kính ranh giới $r$.
* **Mục đích:**
  * Giảm tải hàng trăm ngàn điểm quét thô từ LiDAR/RGB-D camera xuống còn vài ngàn điểm đặc trưng mà vẫn giữ nguyên hình dạng vật thể, giúp các thuật toán Mapping và SLAM chạy mượt mà theo thời gian thực (Real-time).

---

### 9. `DistanceMap` (Bản đồ trường khoảng cách - SDF và UDF)
* **File mã nguồn:** [`distance_map.py`](file:///d:/Document/Robotic-Algorithms/Mapping/DistanceMap/distance_map.py)
* **Nguyên lý hoạt động:**
  * Nhận đầu vào là một ma trận nhị phân (0: vùng trống, 1: vật cản) và chuyển đổi thành bản đồ khoảng cách liên tục thông qua thuật toán biến đổi khoảng cách Euclidean (Euclidean Distance Transform - EDT):
  * **Unsigned Distance Field (UDF):** Tính khoảng cách Euclidean ngắn nhất từ tâm mỗi ô lưới đến biên vật cản gần nhất (luôn $\ge 0$).
  * **Signed Distance Field (SDF):** 
    * Ô nằm ở vùng trống (Free space): mang giá trị **dương (+)**, thể hiện cự ly an toàn tới vật cản.
    * Ô nằm bên trong vật cản (Obstacle): mang giá trị **âm (-)**, thể hiện độ lún sâu vào bên trong vật cản.
  * Trong code, tác giả so sánh giữa việc tự cài đặt thuật toán EDT cổ điển (mất ~3 giây cho map $500 \times 500$) và sử dụng tối ưu `scipy.ndimage.distance_transform_edt` (chỉ mất ~0.05 giây, nhanh hơn 60 lần).
* **Mục đích:**
  * Cực kỳ quan trọng cho khâu **Lập kế hoạch quỹ đạo và Tránh va chạm (Path Planning & Collision Avoidance)**: Cung cấp trực tiếp gradient (hướng đẩy) và chi phí an toàn (Costmap/Potential Field) cho các thuật toán điều khiển như Trajectory Optimization, DWA hoặc MPC.