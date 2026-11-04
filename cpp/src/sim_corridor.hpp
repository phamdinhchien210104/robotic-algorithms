#pragma once
#include <vector>
#include <cmath>
#include <string>

// Kịch bản mô phỏng hành lang bệnh viện (Tương đồng 100% với sim_corridor.py)
struct Wall {
    double x1, y1, x2, y2;
};

struct RobotPose {
    double x, y, theta;
};

struct Pedestrian {
    bool exists;
    double x, y, radius;
};

class SimulatedCorridor {
public:
    std::string scenario;
    double length;
    double width;
    std::vector<Wall> static_walls;

    SimulatedCorridor(std::string scen = "crossing", double len = 25.0, double wid = 3.5)
        : scenario(scen), width(wid) {
        length = (scenario == "long_corridor") ? 60.0 : len;
        static_walls = {
            {0.0, 0.0, length, 0.0},
            {0.0, width, length, width},
            {0.0, 0.0, 0.0, width},
            {length, 0.0, length, width}
        };
    }

    RobotPose get_robot_pose(double t) const {
        double rx = 2.0 + 0.5 * t;
        if (rx > length - 2.0) rx = length - 2.0;
        return {rx, width / 2.0, 0.0};
    }

    Pedestrian get_pedestrian_pos(double t) const {
        if (scenario == "crossing") {
            if (t >= 3.0 && t <= 6.2) {
                double dt = t - 3.0;
                double py = 0.2 + 1.0 * dt;
                double px = 10.0 + 0.05 * std::sin(dt * 2.0);
                return {true, px, py, 0.3};
            }
        }
        return {false, 0.0, 0.0, 0.0};
    }
};

class VirtualLidar {
public:
    int num_rays;
    double max_range;
    std::vector<double> angles;

    VirtualLidar(int rays = 360, double range = 12.0) : num_rays(rays), max_range(range) {
        angles.resize(num_rays);
        for (int i = 0; i < num_rays; i++) {
            angles[i] = -M_PI + (2.0 * M_PI * i) / num_rays;
        }
    }

    void scan(const SimulatedCorridor& corridor, double rx, double ry, double r_theta, double t,
              std::vector<float>& hit_x, std::vector<float>& hit_y,
              std::vector<float>& ray_angles, std::vector<float>& ranges) const {
        hit_x.clear();
        hit_y.clear();
        ray_angles.clear();
        ranges.clear();

        Pedestrian ped = corridor.get_pedestrian_pos(t);

        for (int i = 0; i < num_rays; i++) {
            double global_angle = r_theta + angles[i];
            double cos_a = std::cos(global_angle);
            double sin_a = std::sin(global_angle);

            double min_dist = max_range;

            // 1. Tường tĩnh
            for (const auto& w : corridor.static_walls) {
                double d = ray_segment_intersect(rx, ry, cos_a, sin_a, w.x1, w.y1, w.x2, w.y2);
                if (d > 0.0 && d < min_dist) {
                    min_dist = d;
                }
            }

            // 2. Người đi bộ
            if (ped.exists) {
                double d_ped = ray_circle_intersect(rx, ry, cos_a, sin_a, ped.x, ped.y, ped.radius);
                if (d_ped > 0.0 && d_ped < min_dist) {
                    min_dist = d_ped;
                }
            }

            if (min_dist < max_range) {
                hit_x.push_back(static_cast<float>(rx + min_dist * cos_a));
                hit_y.push_back(static_cast<float>(ry + min_dist * sin_a));
                ray_angles.push_back(static_cast<float>(angles[i]));
                ranges.push_back(static_cast<float>(min_dist));
            } else {
                ranges.push_back(static_cast<float>(max_range));
                ray_angles.push_back(static_cast<float>(angles[i]));
            }
        }
    }

private:
    static double ray_segment_intersect(double rx, double ry, double cos_a, double sin_a,
                                        double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double denom = cos_a * dy - sin_a * dx;
        if (std::abs(denom) < 1e-6) return -1.0;
        double t = ((x1 - rx) * dy - (y1 - ry) * dx) / denom;
        double u = ((x1 - rx) * sin_a - (y1 - ry) * cos_a) / denom;
        if (t > 0.05 && u >= 0.0 && u <= 1.0) return t;
        return -1.0;
    }

    static double ray_circle_intersect(double rx, double ry, double cos_a, double sin_a,
                                       double cx, double cy, double r) {
        double fx = rx - cx;
        double fy = ry - cy;
        double b = 2.0 * (fx * cos_a + fy * sin_a);
        double c = fx * fx + fy * fy - r * r;
        double disc = b * b - 4.0 * c;
        if (disc >= 0) {
            double sq = std::sqrt(disc);
            double t1 = (-b - sq) / 2.0;
            double t2 = (-b + sq) / 2.0;
            if (t1 > 0.05) return t1;
            if (t2 > 0.05) return t2;
        }
        return -1.0;
    }
};
