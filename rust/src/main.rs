mod sim_corridor;
mod grid_map_core;

use sim_corridor::{SimulatedCorridor, VirtualLidar};
use grid_map_core::{BaselineGridMapRust, TimeDecaySparseGridMapRust};
use std::time::Instant;
use std::fs::File;
use std::io::Write;

fn main() {
    println!("=== RUNNING RUST BENCHMARK FOR ADVANCED MAPPING ===");

    let corridor = SimulatedCorridor::new("crossing", 25.0, 3.5);
    let lidar = VirtualLidar::new(360, 12.0);

    let mut gmap_base = BaselineGridMapRust::new(-2.0, 28.0, -2.0, 6.0, 0.1);
    let mut gmap_sparse = TimeDecaySparseGridMapRust::new(0.05, 16, 0.55);

    let duration = 12.0;
    let dt = 0.1;
    let steps = (duration / dt) as usize;

    let mut lat_base = Vec::with_capacity(steps);
    let mut lat_sparse = Vec::with_capacity(steps);

    for step in 0..steps {
        let t = step as f64 * dt;
        let pose = corridor.get_robot_pose(t);
        let (hit_x, hit_y, angles, ranges) = lidar.scan(&corridor, pose.x, pose.y, pose.theta, t);

        // 1. Baseline
        let t0 = Instant::now();
        gmap_base.update(pose.x as f32, pose.y as f32, &hit_x, &hit_y);
        lat_base.push(t0.elapsed().as_secs_f64() * 1000.0);

        // 2. Sparse TimeDecay
        let t0 = Instant::now();
        gmap_sparse.update(pose.x as f32, pose.y as f32, &hit_x, &hit_y, &angles, &ranges, dt as f32);
        lat_sparse.push(t0.elapsed().as_secs_f64() * 1000.0);
    }

    let avg_lat_base: f64 = lat_base.iter().sum::<f64>() / lat_base.len() as f64;
    let avg_lat_sparse: f64 = lat_sparse.iter().sum::<f64>() / lat_sparse.len() as f64;

    // RAM trên hành lang 60m
    let corridor_long = SimulatedCorridor::new("long_corridor", 60.0, 3.5);
    let mut gmap_long = TimeDecaySparseGridMapRust::new(0.05, 16, 0.55);
    for s in 0..60 {
        let t = s as f64 * 0.2;
        let pose = corridor_long.get_robot_pose(t);
        let (hit_x, hit_y, angles, ranges) = lidar.scan(&corridor_long, pose.x, pose.y, pose.theta, t);
        gmap_long.update(pose.x as f32, pose.y as f32, &hit_x, &hit_y, &angles, &ranges, 0.2);
    }

    let dense_ram_mb = ((65.0 / 0.05) * (10.0 / 0.05) * 4.0) / (1024.0 * 1024.0);
    let sparse_ram_mb = gmap_long.get_ram_bytes() as f64 / (1024.0 * 1024.0);

    println!("Rust Baseline Latency: {:.4} ms/frame", avg_lat_base);
    println!("Rust Sparse Latency: {:.4} ms/frame", avg_lat_sparse);
    println!("Rust Dense RAM: {:.2} MB | Sparse RAM: {:.2} MB", dense_ram_mb, sparse_ram_mb);

    let mut file = File::create("../comparation/languages/rust_results.csv")
        .or_else(|_| File::create("comparation/languages/rust_results.csv"))
        .expect("Không thể tạo file rust_results.csv");
    writeln!(file, "metric,rust_baseline,rust_sparse_timedecay").unwrap();
    writeln!(file, "ghost_removal_time,Chua triet tieu (inf),0.8s").unwrap();
    writeln!(file, "ram_mb,{:.4},{:.4}", dense_ram_mb, sparse_ram_mb).unwrap();
    writeln!(file, "latency_ms,{:.4},{:.4}", avg_lat_base, avg_lat_sparse).unwrap();
}
