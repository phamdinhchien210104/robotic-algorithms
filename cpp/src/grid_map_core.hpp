#pragma once
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <cstdint>

// Bresenham 2D integer line
inline void bresenham_line(int x0, int y0, int x1, int y1, std::vector<std::pair<int, int>>& line) {
    line.clear();
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        line.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// 1. Baseline C++ (Dense Matrix 2D + Log-Odds tĩnh)
class BaselineGridMapCpp {
public:
    double min_x, max_x, min_y, max_y, resolution;
    int width, height;
    std::vector<float> log_odds;
    float l_occ, l_free, l_min, l_max;

    BaselineGridMapCpp(double mx = -2.0, double xx = 28.0, double my = -2.0, double xy = 6.0, double res = 0.1)
        : min_x(mx), max_x(xx), min_y(my), max_y(xy), resolution(res) {
        width = static_cast<int>(std::round((max_x - min_x) / resolution));
        height = static_cast<int>(std::round((max_y - min_y) / resolution));
        log_odds.assign(width * height, 0.0f);
        l_occ = 0.85f;
        l_free = -0.40f;
        l_min = -5.0f;
        l_max = 5.0f;
    }

    void update(float rx, float ry, const std::vector<float>& hit_x, const std::vector<float>& hit_y) {
        int r_ix = static_cast<int>(std::round((rx - min_x) / resolution));
        int r_iy = static_cast<int>(std::round((ry - min_y) / resolution));

        std::vector<std::pair<int, int>> line;
        for (size_t i = 0; i < hit_x.size(); i++) {
            int h_ix = static_cast<int>(std::round((hit_x[i] - min_x) / resolution));
            int h_iy = static_cast<int>(std::round((hit_y[i] - min_y) / resolution));

            if (h_ix < 0 || h_ix >= width || h_iy < 0 || h_iy >= height) continue;

            bresenham_line(r_ix, r_iy, h_ix, h_iy, line);
            for (size_t j = 0; j < line.size() - 1; j++) {
                int px = line[j].first;
                int py = line[j].second;
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    int idx = px * height + py;
                    log_odds[idx] = std::max(l_min, log_odds[idx] + l_free);
                }
            }
            int h_idx = h_ix * height + h_iy;
            log_odds[h_idx] = std::min(l_max, log_odds[h_idx] + l_occ);
        }
    }

    int count_ghost_cells() const {
        int count = 0;
        int ix1 = static_cast<int>(std::round((9.2 - min_x) / resolution));
        int iy1 = static_cast<int>(std::round((0.6 - min_y) / resolution));
        int ix2 = static_cast<int>(std::round((10.8 - min_x) / resolution));
        int iy2 = static_cast<int>(std::round((2.9 - min_y) / resolution));

        for (int x = std::min(ix1, ix2); x <= std::max(ix1, ix2); x++) {
            for (int y = std::min(iy1, iy2); y <= std::max(iy1, iy2); y++) {
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    float p = 1.0f / (1.0f + std::exp(-log_odds[x * height + y]));
                    if (p > 0.65f) count++;
                }
            }
        }
        return count;
    }
};

// 2. TimeDecay Chunk Sparse C++
struct Chunk {
    float data[16][16];
    Chunk() {
        for (int i = 0; i < 16; i++)
            for (int j = 0; j < 16; j++)
                data[i][j] = 0.0f;
    }
};

class TimeDecaySparseGridMapCpp {
public:
    double resolution;
    int chunk_size;
    float decay_rate;
    std::unordered_map<uint64_t, Chunk> chunks;
    float l_occ, l_free, l_min, l_max;

    TimeDecaySparseGridMapCpp(double res = 0.05, int cs = 16, float decay = 0.55f)
        : resolution(res), chunk_size(cs), decay_rate(decay) {
        l_occ = 0.85f;
        l_free = -0.40f;
        l_min = -5.0f;
        l_max = 5.0f;
    }

    static uint64_t hash_key(int cx, int cy) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
               (static_cast<uint64_t>(static_cast<uint32_t>(cy)));
    }

    void update(float rx, float ry, const std::vector<float>& hit_x, const std::vector<float>& hit_y,
                const std::vector<float>& angles, const std::vector<float>& ranges, float dt = 0.1f) {
        float decay_factor = std::exp(-decay_rate * dt);

        // 1. Decay & Garbage collection
        for (auto it = chunks.begin(); it != chunks.end(); ) {
            float max_v = 0.0f;
            for (int i = 0; i < 16; i++) {
                for (int j = 0; j < 16; j++) {
                    it->second.data[i][j] *= decay_factor;
                    float abs_v = std::abs(it->second.data[i][j]);
                    if (abs_v > max_v) max_v = abs_v;
                }
            }
            if (max_v < 0.05f) {
                it = chunks.erase(it);
            } else {
                ++it;
            }
        }

        // 2. Cập nhật ray steps
        float inv_res = 1.0f / static_cast<float>(resolution);
        for (size_t r = 0; r < ranges.size(); r++) {
            float dist = ranges[r];
            if (dist < 0.15f) continue;
            float cos_a = std::cos(angles[r]);
            float sin_a = std::sin(angles[r]);

            for (float d = 0.1f; d < dist - 0.15f; d += 0.15f) {
                float fx = rx + d * cos_a;
                float fy = ry + d * sin_a;
                int ix = static_cast<int>(std::round(fx * inv_res));
                int iy = static_cast<int>(std::round(fy * inv_res));
                int cx = ix / chunk_size;
                int cy = iy / chunk_size;
                int lx = (ix % chunk_size + chunk_size) % chunk_size;
                int ly = (iy % chunk_size + chunk_size) % chunk_size;

                uint64_t k = hash_key(cx, cy);
                float val = chunks[k].data[lx][ly] + l_free;
                chunks[k].data[lx][ly] = std::max(l_min, val);
            }
        }

        // 3. Occupied hits
        for (size_t i = 0; i < hit_x.size(); i++) {
            int ix = static_cast<int>(std::round(hit_x[i] * inv_res));
            int iy = static_cast<int>(std::round(hit_y[i] * inv_res));
            int cx = ix / chunk_size;
            int cy = iy / chunk_size;
            int lx = (ix % chunk_size + chunk_size) % chunk_size;
            int ly = (iy % chunk_size + chunk_size) % chunk_size;

            uint64_t k = hash_key(cx, cy);
            float val = chunks[k].data[lx][ly] + l_occ;
            chunks[k].data[lx][ly] = std::min(l_max, val);
        }
    }

    size_t get_ram_bytes() const {
        return chunks.size() * (sizeof(Chunk) + 32);
    }

    int count_ghost_cells() const {
        int count = 0;
        float inv_res = 1.0f / static_cast<float>(resolution);
        int ix1 = static_cast<int>(std::round(9.2 * inv_res));
        int iy1 = static_cast<int>(std::round(0.6 * inv_res));
        int ix2 = static_cast<int>(std::round(10.8 * inv_res));
        int iy2 = static_cast<int>(std::round(2.9 * inv_res));

        for (int x = std::min(ix1, ix2); x <= std::max(ix1, ix2); x++) {
            for (int y = std::min(iy1, iy2); y <= std::max(iy1, iy2); y++) {
                int cx = x / chunk_size;
                int cy = y / chunk_size;
                uint64_t k = hash_key(cx, cy);
                auto it = chunks.find(k);
                if (it != chunks.end()) {
                    int lx = (x % chunk_size + chunk_size) % chunk_size;
                    int ly = (y % chunk_size + chunk_size) % chunk_size;
                    float p = 1.0f / (1.0f + std::exp(-it->second.data[lx][ly]));
                    if (p > 0.65f) count++;
                }
            }
        }
        return count;
    }
};
