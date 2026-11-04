// grid_map_core.rs - Triển khai lõi Mapping trong Rust
use std::collections::HashMap;

// Bresenham line algorithm in Rust
pub fn bresenham_line(mut x0: i32, mut y0: i32, x1: i32, y1: i32) -> Vec<(i32, i32)> {
    let mut line = Vec::new();
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;

    loop {
        line.push((x0, y0));
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x0 += sx;
        }
        if e2 < dx {
            err += dx;
            y0 += sy;
        }
    }
    line
}

// 1. Baseline Grid Map (Rust)
pub struct BaselineGridMapRust {
    pub min_x: f64,
    pub min_y: f64,
    pub resolution: f64,
    pub width: usize,
    pub height: usize,
    pub log_odds: Vec<f32>,
}

impl BaselineGridMapRust {
    pub fn new(min_x: f64, max_x: f64, min_y: f64, max_y: f64, resolution: f64) -> Self {
        let width = ((max_x - min_x) / resolution).round() as usize;
        let height = ((max_y - min_y) / resolution).round() as usize;
        BaselineGridMapRust {
            min_x,
            min_y,
            resolution,
            width,
            height,
            log_odds: vec![0.0; width * height],
        }
    }

    pub fn update(&mut self, rx: f32, ry: f32, hit_x: &[f32], hit_y: &[f32]) {
        let r_ix = ((rx as f64 - self.min_x) / self.resolution).round() as i32;
        let r_iy = ((ry as f64 - self.min_y) / self.resolution).round() as i32;

        for i in 0..hit_x.len() {
            let h_ix = ((hit_x[i] as f64 - self.min_x) / self.resolution).round() as i32;
            let h_iy = ((hit_y[i] as f64 - self.min_y) / self.resolution).round() as i32;

            if h_ix < 0 || h_ix >= self.width as i32 || h_iy < 0 || h_iy >= self.height as i32 {
                continue;
            }

            let line = bresenham_line(r_ix, r_iy, h_ix, h_iy);
            for &(px, py) in &line[..line.len().saturating_sub(1)] {
                if px >= 0 && px < self.width as i32 && py >= 0 && py < self.height as i32 {
                    let idx = px as usize * self.height + py as usize;
                    self.log_odds[idx] = (self.log_odds[idx] - 0.4).max(-5.0);
                }
            }

            let h_idx = h_ix as usize * self.height + h_iy as usize;
            self.log_odds[h_idx] = (self.log_odds[h_idx] + 0.85).min(5.0);
        }
    }
}

// 2. Chunk Sparse TimeDecay (Rust)
pub struct Chunk {
    pub data: [[f32; 16]; 16],
}

impl Chunk {
    pub fn new() -> Self {
        Chunk { data: [[0.0; 16]; 16] }
    }
}

pub struct TimeDecaySparseGridMapRust {
    pub resolution: f64,
    pub chunk_size: usize,
    pub decay_rate: f32,
    pub chunks: HashMap<(i32, i32), Chunk>,
}

impl TimeDecaySparseGridMapRust {
    pub fn new(resolution: f64, chunk_size: usize, decay_rate: f32) -> Self {
        TimeDecaySparseGridMapRust {
            resolution,
            chunk_size,
            decay_rate,
            chunks: HashMap::new(),
        }
    }

    pub fn update(
        &mut self,
        rx: f32,
        ry: f32,
        hit_x: &[f32],
        hit_y: &[f32],
        angles: &[f32],
        ranges: &[f32],
        dt: f32,
    ) {
        let decay_factor = (-self.decay_rate * dt).exp();

        // 1. Decay & GC
        self.chunks.retain(|_, chunk| {
            let mut max_v: f32 = 0.0;
            for i in 0..16 {
                for j in 0..16 {
                    chunk.data[i][j] *= decay_factor;
                    let abs_v = chunk.data[i][j].abs();
                    if abs_v > max_v {
                        max_v = abs_v;
                    }
                }
            }
            max_v >= 0.05
        });

        // 2. Ray marching free space
        let inv_res = 1.0 / self.resolution as f32;
        let cs = self.chunk_size as i32;

        for i in 0..ranges.len() {
            let dist = ranges[i];
            if dist < 0.15 {
                continue;
            }
            let cos_a = angles[i].cos();
            let sin_a = angles[i].sin();

            let mut d = 0.1;
            while d < dist - 0.15 {
                let fx = rx + d * cos_a;
                let fy = ry + d * sin_a;
                let ix = (fx * inv_res).round() as i32;
                let iy = (fy * inv_res).round() as i32;
                let cx = ix.div_euclid(cs);
                let cy = iy.div_euclid(cs);
                let lx = ix.rem_euclid(cs) as usize;
                let ly = iy.rem_euclid(cs) as usize;

                let chunk = self.chunks.entry((cx, cy)).or_insert_with(Chunk::new);
                chunk.data[lx][ly] = (chunk.data[lx][ly] - 0.4).max(-5.0);

                d += 0.15;
            }
        }

        // 3. Occupied hits
        for i in 0..hit_x.len() {
            let ix = (hit_x[i] * inv_res).round() as i32;
            let iy = (hit_y[i] * inv_res).round() as i32;
            let cx = ix.div_euclid(cs);
            let cy = iy.div_euclid(cs);
            let lx = ix.rem_euclid(cs) as usize;
            let ly = iy.rem_euclid(cs) as usize;

            let chunk = self.chunks.entry((cx, cy)).or_insert_with(Chunk::new);
            chunk.data[lx][ly] = (chunk.data[lx][ly] + 0.85).min(5.0);
        }
    }

    pub fn get_ram_bytes(&self) -> usize {
        self.chunks.len() * (std::mem::size_of::<Chunk>() + 32)
    }
}
