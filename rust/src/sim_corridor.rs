// sim_corridor.rs - Mô phỏng hành lang & LiDAR 2D trong Rust
use std::f64::consts::PI;

pub struct Wall {
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
}

pub struct RobotPose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

pub struct Pedestrian {
    pub exists: bool,
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

pub struct SimulatedCorridor {
    pub scenario: String,
    pub length: f64,
    pub width: f64,
    pub static_walls: Vec<Wall>,
}

impl SimulatedCorridor {
    pub fn new(scenario: &str, length: f64, width: f64) -> Self {
        let actual_len = if scenario == "long_corridor" { 60.0 } else { length };
        let walls = vec![
            Wall { x1: 0.0, y1: 0.0, x2: actual_len, y2: 0.0 },
            Wall { x1: 0.0, y1: width, x2: actual_len, y2: width },
            Wall { x1: 0.0, y1: 0.0, x2: 0.0, y2: width },
            Wall { x1: actual_len, y1: 0.0, x2: actual_len, y2: width },
        ];
        SimulatedCorridor {
            scenario: scenario.to_string(),
            length: actual_len,
            width,
            static_walls: walls,
        }
    }

    pub fn get_robot_pose(&self, t: f64) -> RobotPose {
        let mut rx = 2.0 + 0.5 * t;
        if rx > self.length - 2.0 {
            rx = self.length - 2.0;
        }
        RobotPose { x: rx, y: self.width / 2.0, theta: 0.0 }
    }

    pub fn get_pedestrian_pos(&self, t: f64) -> Pedestrian {
        if self.scenario == "crossing" && (3.0..=6.2).contains(&t) {
            let dt = t - 3.0;
            let py = 0.2 + 1.0 * dt;
            let px = 10.0 + 0.05 * (dt * 2.0).sin();
            return Pedestrian { exists: true, x: px, y: py, radius: 0.3 };
        }
        Pedestrian { exists: false, x: 0.0, y: 0.0, radius: 0.0 }
    }
}

pub struct VirtualLidar {
    pub num_rays: usize,
    pub max_range: f64,
    pub angles: Vec<f64>,
}

impl VirtualLidar {
    pub fn new(num_rays: usize, max_range: f64) -> Self {
        let mut angles = Vec::with_capacity(num_rays);
        for i in 0..num_rays {
            angles.push(-PI + (2.0 * PI * i as f64) / num_rays as f64);
        }
        VirtualLidar { num_rays, max_range, angles }
    }

    pub fn scan(
        &self,
        corridor: &SimulatedCorridor,
        rx: f64,
        ry: f64,
        r_theta: f64,
        t: f64,
    ) -> (Vec<f32>, Vec<f32>, Vec<f32>, Vec<f32>) {
        let mut hit_x = Vec::with_capacity(self.num_rays);
        let mut hit_y = Vec::with_capacity(self.num_rays);
        let mut ray_angles = Vec::with_capacity(self.num_rays);
        let mut ranges = Vec::with_capacity(self.num_rays);

        let ped = corridor.get_pedestrian_pos(t);

        for &ang in &self.angles {
            let global_a = r_theta + ang;
            let cos_a = global_a.cos();
            let sin_a = global_a.sin();
            let mut min_d = self.max_range;

            for w in &corridor.static_walls {
                let d = ray_segment_intersect(rx, ry, cos_a, sin_a, w.x1, w.y1, w.x2, w.y2);
                if d > 0.0 && d < min_d {
                    min_d = d;
                }
            }

            if ped.exists {
                let d = ray_circle_intersect(rx, ry, cos_a, sin_a, ped.x, ped.y, ped.radius);
                if d > 0.0 && d < min_d {
                    min_d = d;
                }
            }

            if min_d < self.max_range {
                hit_x.push((rx + min_d * cos_a) as f32);
                hit_y.push((ry + min_d * sin_a) as f32);
                ranges.push(min_d as f32);
            } else {
                ranges.push(self.max_range as f32);
            }
            ray_angles.push(ang as f32);
        }

        (hit_x, hit_y, ray_angles, ranges)
    }
}

fn ray_segment_intersect(rx: f64, ry: f64, cos_a: f64, sin_a: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let denom = cos_a * dy - sin_a * dx;
    if denom.abs() < 1e-6 {
        return -1.0;
    }
    let t = ((x1 - rx) * dy - (y1 - ry) * dx) / denom;
    let u = ((x1 - rx) * sin_a - (y1 - ry) * cos_a) / denom;
    if t > 0.05 && (0.0..=1.0).contains(&u) {
        t
    } else {
        -1.0
    }
}

fn ray_circle_intersect(rx: f64, ry: f64, cos_a: f64, sin_a: f64, cx: f64, cy: f64, r: f64) -> f64 {
    let fx = rx - cx;
    let fy = ry - cy;
    let b = 2.0 * (fx * cos_a + fy * sin_a);
    let c = fx * fx + fy * fy - r * r;
    let disc = b * b - 4.0 * c;
    if disc >= 0.0 {
        let sq = disc.sqrt();
        let t1 = (-b - sq) / 2.0;
        let t2 = (-b + sq) / 2.0;
        if t1 > 0.05 { return t1; }
        if t2 > 0.05 { return t2; }
    }
    -1.0
}
