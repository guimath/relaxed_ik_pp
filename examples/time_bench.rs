use clap::Parser;
use relaxed_ik_lib::relaxed_ik::RelaxedIK;
use std::fs;
use std::io::Write;
use std::{io, path::PathBuf, time};
use csv::Writer;
/// args
#[derive(Parser)]
struct Cli {
    /// Specify path to the settings file.
    #[arg(
        short,
        long,
        default_value = "configs/ur5.toml",
        value_name = "FILE PATH"
    )]
    settings: PathBuf,

    /// Specify name of csv save file.
    #[arg(
        short,
        long,
        value_name = "FILE NAME",
        default_value = "time_bench_data.csv"
    )]
    file_name: String,
}

use serde::Serialize;

#[derive(Debug, Serialize)]
struct Record {
    x: f64,
    y: f64,
    z: f64,
    single_ik: u64,
    double_ik: u64,
    motion_alone: Option<u64>,
    // motion_full: Option<u64>,
}

fn main() {
    env_logger::init();
    let args = Cli::parse();

    let folder = args.settings.file_stem().unwrap().to_str().unwrap();
    let file_name = args.file_name;
    let mut data_file = PathBuf::from(format!("ex_out/{folder}/data/{file_name}"));
    let _ = fs::create_dir_all(data_file.parent().unwrap());

    data_file.set_extension("csv");

    // rik init
    let mut rik = RelaxedIK::new(args.settings.to_str().unwrap());
    let mut shapes = rik.planner.obstacles.shapes().to_vec();

    let start = [0.05, -1.0, 0.3];
    let stop = [1.0, 1.0, 0.6];
    let sample_num = [100usize, 200, 5];

    let step = [
        (stop[0] - start[0]) / sample_num[0] as f64,
        (stop[1] - start[1]) / sample_num[1] as f64,
        (stop[2] - start[2]) / sample_num[2] as f64,
    ];
    println!(
        "Number of calls: {:}",
        sample_num[0] * sample_num[1] * sample_num[2]
    );

    let mut target: [f64; 3];
    let (mut x, mut y, mut z): (f64, f64, f64);
    let mut wtr = Writer::from_path(data_file).unwrap();
    let global_start = time::Instant::now();
    let mut time_est: f64;
    for i in 0..sample_num[0] {
        x = start[0] + step[0] * i as f64;
        time_est = (time::Instant::now() - global_start.clone()).as_secs_f64()
            * (sample_num[0] - i) as f64
            / i as f64;
        for j in 0..sample_num[1] {
            y = start[1] + step[1] * j as f64;
            let radius = x.powi(2) + y.powi(2);
            if radius < 0.2 * 0.2 || radius > 1.0 * 1.0 {
                continue;
            }
            print!(
                "\r{i:4}/{} - {j:4}/{} - ({x:+.3}, {y:+.3}) - Estimated time left {time_est:4.0}s",
                sample_num[0], sample_num[1]
            );
            io::stdout().flush().unwrap();
            for k in 0..sample_num[2] {
                z = start[2] + step[2] * k as f64;
                target = [x, y, z];
                shapes[0].0.translation =
                    nalgebra::Translation3::new(target[0], target[1], target[2]);
                let compound = ncollide3d::shape::Compound::new(shapes.clone());
                rik.planner.obstacles = compound;

                rik.reset_origin();
                let t1 = time::Instant::now();
                let _ = rik.repeat_solve_ik(target);
                let single_ik = (time::Instant::now() - t1).as_micros() as u64;

                rik.reset_origin();
                let t1 = time::Instant::now();
                let r = rik.grip_two_ik(target);
                let double_ik = (time::Instant::now() - t1).as_micros() as u64;

                let motion_alone = match r {
                    Ok((x0, x1, _x2, _)) => {
                        // rik.reset_origin();
                        let t1 = time::Instant::now();
                        let q = rik.planner.get_motion(x0, x1.clone());
                        // let q2 = rik.planner.get_motion(x1, x2); //[1..]; // first alread
                        let dur = (time::Instant::now() - t1).as_micros() as u64;
                        if q.is_err() {
                            None
                        } else {
                            Some(dur)
                        }
                    }
                    Err(_) => None,
                };

                // let motion_full: Option<u64>;
                // if motion_alone.is_some() {
                //     rik.reset_origin();
                //     let t1 = time::Instant::now();
                //     let _ = rik.grip(target);
                //     motion_full = Some((time::Instant::now() - t1).as_micros() as u64);
                // } else {
                //     motion_full = None
                // }
                wtr.serialize(Record {
                    x,
                    y,
                    z,
                    single_ik,
                    double_ik,
                    motion_alone,
                })
                .unwrap();
            }
        }
    }
    wtr.flush().unwrap();
}
