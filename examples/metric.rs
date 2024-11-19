use clap::Parser;
use relaxed_ik_lib::relaxed_ik::RelaxedIK;
use relaxed_ik_lib::Error;
use savefile::prelude::*;
use serde::Serialize;
use std::io::{self, Write};
use std::path::PathBuf;
use std::time::Duration;
use std::{fs, time};

use indicatif::{MultiProgress, ProgressBar};

#[derive(clap::ValueEnum, Clone, Default, Debug, Serialize)]
#[serde(rename_all = "kebab-case")]
enum ScanMode {
    /// compute inverse kinematics  
    #[default]
    IK,
    /// compute motion
    Motion,
}

/// args
#[derive(Parser)]
struct Cli {
    /// Specify path to the settings file
    #[arg(
        short,
        long,
        default_value = "configs/ur5_grip.yaml",
        value_name = "FILE PATH"
    )]
    settings: PathBuf,
    /// Scan mode
    #[clap(short, long, default_value_t, value_enum)]
    mode: ScanMode,
    /// x range
    #[arg(short, long, value_names=&["MIN X", "MAX X"], num_args = 2, default_values_t=&[-1.3, 1.3],  allow_hyphen_values=true)]
    x_zone: Vec<f64>,
    /// y range
    #[arg(short, long, value_names=&["MIN Y", "MAX Y"], num_args = 2, default_values_t=&[-1.3, 1.3],  allow_hyphen_values=true)]
    y_zone: Vec<f64>,
    /// Height at which to scan
    #[clap(short, long, default_value_t = 0.3)]
    z_target: f64,
    /// Number of sample per axis
    #[clap(long, default_value_t = 200, value_name = "NUMBER")]
    sample_per_axis: usize,
    /// name of the scan data file - Default is <MODE>_Z_<Z_VALUE>.bin
    #[arg(short, long, value_name = "FILE NAME")]
    data_file: Option<String>,
    /// Radius of range of motion (1.1 for UR5, 1.0 for Xarm6) - Any point out of radius will be ignored
    #[clap(short, long, default_value_t = 1.0)]
    range_max_radius: f64,
}

#[derive(Serialize, Debug, Default)]
struct GraphConfig {
    x_zone: [f64; 2],
    y_zone: [f64; 2],
    z_target: f64,
    compute_time_s: f64,
    compute_num: u32,
    scale_label: String,
    range_scale: Option<[f64; 2]>,
    points_scale: Option<usize>,
    data_offset: Option<f64>,
}

fn q_to_dist(q1: Vec<Vec<f64>>, q2: Vec<Vec<f64>>) -> f64 {
    //(q1.len() + q2.len()) as f64
    let dof = q1[0].len();
    let mut delta = 0.0;
    let mut q = q1;
    q.extend(q2);
    for i in 1..q.len() {
        for j in 0..dof {
            delta += (q[i][j] - q[i - 1][j]).abs();
        }
    }
    delta
}

fn q_to_value(q1: Vec<Vec<f64>>, q2: Vec<Vec<f64>>, get_dist:bool) -> f64 {
    if get_dist{
        q_to_dist(q1, q2)
    }
    else {
        (q1.len() + q2.len()) as f64
    }
}

fn write_toml(graph_config_file:PathBuf, graph_config:&GraphConfig){
    let toml = toml::to_string(graph_config).unwrap();
    fs::write(graph_config_file, toml).expect("Could not write to graph_config file");
}

fn main() {
    env_logger::init();
    let args = Cli::parse();

    let start_x = args.x_zone[0];
    let start_y = args.y_zone[0];

    let step_x = (args.x_zone[1] - args.x_zone[0]) / (args.sample_per_axis as f64);
    let step_y = (args.y_zone[1] - args.y_zone[0]) / (args.sample_per_axis as f64);

    let avg_time_one_calc = match args.mode {
        ScanMode::IK => 0.9f64,
        ScanMode::Motion => 9.0f64,
    };
    let total_test = args.sample_per_axis * args.sample_per_axis;
    let est_time = Duration::from_millis((total_test as f64 * avg_time_one_calc) as u64);
    println!(
        "Total tests planned {:} ({:}x{:}). Estimated time : {:.3?}",
        total_test, args.sample_per_axis, args.sample_per_axis, est_time
    );

    // out file names
    let folder = args.settings.file_stem().unwrap().to_str().unwrap();
    let file_name = args
        .data_file
        .unwrap_or(format!("{:?}_Z_{:+}.bin", args.mode, args.z_target));
    let mut data_file = PathBuf::from(format!("ex_out/{folder}/data/{file_name}"));
    data_file.set_extension("bin");
    let _ = fs::create_dir_all(data_file.parent().unwrap());
    if fs::metadata(data_file.clone()).is_ok() {
        print!(
            "File {:#?} already exists, replace ? (Y/n): ",
            data_file.as_os_str()
        );
        io::stdout().flush().unwrap();
        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        let response = input.trim().to_uppercase();
        if response != "Y" {
            return;
        }
    }

    // rik init
    let mut rik = RelaxedIK::new(args.settings.to_str().unwrap());
    let mut shapes = rik.planner.obstacles.shapes().to_vec();

    let mut graph_config_file = data_file.clone();
    graph_config_file.set_extension("toml");
    let mut graph_config = GraphConfig {
        x_zone: [args.x_zone[0], args.x_zone[1]],
        y_zone: [args.y_zone[0], args.y_zone[1]],
        z_target: args.z_target,
        compute_time_s: 0.0,
        compute_num: 0,
        scale_label: "Cost increase from minimum".to_string(),
        data_offset: Some(-rik.min_possible_cost),
        range_scale: Some([0.0, 50.0]),
        points_scale: None,
    };
    write_toml(graph_config_file.clone(), &graph_config);
    println!("Data will be saved to {:#?}", data_file.as_os_str());
    println!("\tMetadata saved to {:#?}", graph_config_file.as_os_str());

    // Graph init
    let mut matrix = vec![vec![Err(Error::OutOfRange); args.sample_per_axis]; args.sample_per_axis];
    let mut num_calc = 0;

    let m = MultiProgress::new();
    let pb = m.add(ProgressBar::new(args.sample_per_axis as u64));
    let pb2 = m.add(ProgressBar::new(args.sample_per_axis as u64));
    let t1 = time::Instant::now();

    let radius_max_square = args.range_max_radius * args.range_max_radius;
    match args.mode {
        ScanMode::IK => {
            let mut target: [f64; 3];
            for i in 0..args.sample_per_axis {
                for j in 0..args.sample_per_axis {
                    let i_val = start_x + i as f64 * step_x;
                    let j_val = start_y + j as f64 * step_y;
                    target = [
                        i_val,
                        j_val,
                        args.z_target,
                    ];
                    if i_val.powi(2) + j_val.powi(2) > radius_max_square {
                        continue;
                    }
                    num_calc += 1;
                    rik.reset_origin();

                    let res = rik.grip_two_ik(target);
                    matrix[j][i] = match res {
                        Ok((_, _, _, status)) => Ok(status.cost_value()),
                        Err(e) => Err(e),
                    };
                    pb2.set_position(j as u64);
                }
                pb.set_position(i as u64);
                if i % 10 == 0 && i != 0 {
                    save_file(data_file.clone(), 0, &matrix).unwrap();
                }
            }
        }
        ScanMode::Motion => {
            const GET_DIST:bool= false;
            if GET_DIST{
                graph_config.data_offset = None;
                graph_config.range_scale = Some([3.0, 20.0]);
                graph_config.points_scale = None;
                graph_config.scale_label = "C-Space distance to reach grasp pose (rad)".to_string();
            }
            else {
                graph_config.data_offset = Some(-4.0);
                graph_config.range_scale = Some([0.0, 10.0]);
                graph_config.points_scale = Some(10);
                graph_config.scale_label = "Additional steps in motion planning".to_string();
            }
            write_toml(graph_config_file.clone(), &graph_config);

            for i in 0..args.sample_per_axis {
                for j in 0..args.sample_per_axis {
                    let target = [
                        start_x + i as f64 * step_x,
                        start_y + j as f64 * step_y,
                        args.z_target,
                    ];
                    let rad_dist = target[0] * target[0] + target[1] * target[1];
                    if 0.2 * 0.2 > rad_dist || rad_dist > radius_max_square {
                        continue;
                    }
                    num_calc += 1;
                    rik.reset_origin();
                    shapes[0].0.translation = nalgebra::Translation3::new(target[0], target[1], target[2]);
                    let compound = ncollide3d::shape::Compound::new(shapes.clone());
                    rik.planner.obstacles = compound;
                    let grip = rik.grip(target);
                    matrix[j][i] = match grip {
                        Ok((q1, q2, _res)) => Ok(q_to_value(q1, q2, GET_DIST)),
                        Err(e) => Err(e),
                    };
                    pb2.set_position(j as u64);
                }
                pb.set_position(i as u64);
                if i % 5 == 0 && i != 0 {
                    save_file(data_file.clone(), 0, &matrix).unwrap();
                }
            }
        }
    }

    let elapsed = time::Instant::now() - t1;
    pb.finish_and_clear();
    let _ = m.clear();
    println!("\rTotal time elapsed = {elapsed:.3?}");
    println!(
        "Total calculation = {num_calc:} ({:.3?} per calc)",
        elapsed / num_calc
    );
    save_file(data_file, 0, &matrix).unwrap();
    graph_config.compute_num = num_calc;
    graph_config.compute_time_s = elapsed.as_secs_f64();
    write_toml(graph_config_file.clone(), &graph_config);
}
// Cyan: Solver error on inverse kinematics (IK failed NotFiniteComputation)
// White: Ik solved but not satisfying (IK, cost higher then threshold)
// Black: Collision with bottle at start (Collision error: ... (Start))
// Magenta: Collision with bottle at End (Collision error: ... (Goal))
// Red: path planing failed ()