use clap::Parser;
use nalgebra::Translation3;
use ncollide3d::shape::Compound;
use plotters::prelude::*;
use relaxed_ik_lib::relaxed_ik::RelaxedIK;
use serde::Serialize;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::time::Duration;
use std::{fs, time};

use indicatif::{MultiProgress, ProgressBar};
use scarlet::color::RGBColor;
use scarlet::colormap::{ColorMap, GradientColorMap};

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
    /// Height at which to scan
    #[clap(short, long, default_value_t=0.3)]
    z_target: f64,
    /// Radius of range of motion (1.1 for UR5, 1.0 for Xarm6) - Any point out of radius will be ignored
    #[clap(short, long, default_value_t=1.0)]
    range_max_radius: f64,
}

fn err_to_color(err: openrr_planner::Error) -> RGBAColor {
    match err {
        openrr_planner::Error::ParseError { .. } => WHITE.to_rgba(), // cost to high
        openrr_planner::Error::Other { error: _ } => CYAN.to_rgba(), // SolverError
        openrr_planner::Error::Collision {
            point: p,
            collision_link_names: _,
        } => {
            if format!("{p:?}") == "Start" {
                BLACK.to_rgba()
            }
            // UnfeasibleTrajectoryPoint is private :(
            else {
                MAGENTA.to_rgba()
            } // goal collision
        }
        _ => RED.to_rgba(), // Path planning failed
    }
}


fn main() {
    env_logger::init();
    let args = Cli::parse();

    const START_X: f64 = -1.3;
    const START_Y: f64 = -1.3;

    const STEP_X: f64 = 0.01;
    const STEP_Y: f64 = 0.01;
    const MAX_X: f64 = 1.3;
    const MAX_Y: f64 = 1.3;

    const I_MAX: usize = ((MAX_X - START_X) / STEP_X) as usize;
    const J_MAX: usize = ((MAX_Y - START_Y) / STEP_Y) as usize;

    let (avg_time_one_calc, file_name) = match args.mode {
        ScanMode::IK => (0.9f64, format!("ik_Z_{:+}", args.z_target)),
        ScanMode::Motion => (9.0f64, format!("motion_Z_{:+}", args.z_target)),
    };

    let est_time = Duration::from_millis(((I_MAX * J_MAX) as f64 * avg_time_one_calc) as u64);
    println!(
        "Total tests planned {:} ({:}x{:}). Estimated time : {:.3?}",
        I_MAX * J_MAX,
        I_MAX,
        J_MAX,
        est_time
    );

    // out file names
    let folder = args.settings.file_stem().unwrap().to_str().unwrap();
    let log_file = PathBuf::from(format!("ex_out/{folder}/{file_name}.log"));
    let pic_file = PathBuf::from(format!("ex_out/{folder}/{file_name}.png"));

    let _ = fs::create_dir_all(pic_file.parent().unwrap());
    let mut f = File::create(log_file).unwrap();
    println!("Graph will be saved to {:#?}", pic_file.as_os_str());

    // rik init
    let mut rik = RelaxedIK::new(args.settings.to_str().unwrap());
    let mut shapes = rik.planner.obstacles.shapes().to_vec(); 

    // Graph init
    let mut matrix = [[WHITE.to_rgba(); I_MAX]; J_MAX];
    let root = BitMapBackend::new(pic_file.as_os_str(), (1024, 1024)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let mut chart = ChartBuilder::on(&root)
        .caption(format!("{folder} - Z = {:}",args.z_target), ("sans-serif", 40))
        .margin(10)
        .top_x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(0i32..(I_MAX as i32), (J_MAX as i32)..0i32)
        .unwrap();

    chart
        .configure_mesh()
        .x_desc("x")
        .y_desc("y")
        .x_labels(10)
        .y_labels(10)
        .max_light_lines(4)
        .x_label_formatter(&|r| format!("{:.2}", START_X + STEP_X * (*r as f64)))
        .y_label_formatter(&|r| format!("{:.2}", START_Y + STEP_Y * (*r as f64)))
        .disable_x_mesh()
        .disable_y_mesh()
        .label_style(("sans-serif", 20))
        .draw()
        .unwrap();

    let m = MultiProgress::new();
    let pb = m.add(ProgressBar::new(I_MAX as u64));
    let pb2 = m.add(ProgressBar::new(J_MAX as u64));
    let t1 = time::Instant::now();

    let red = RGBColor::from_hex_code("#ff0000").unwrap();
    let green = RGBColor::from_hex_code("#00ff00").unwrap();
    let cmap = GradientColorMap::new_linear(green, red);
    let to_color =  |h| {
        let color = cmap.transform_single(h);
        plotters::prelude::RGBColor(color.int_r(), color.int_g(), color.int_b())
            .to_rgba()
    };

    let radius_max_square = args.range_max_radius*args.range_max_radius;
    match args.mode {
        ScanMode::IK => {
            let min_cost = rik.min_possible_cost;
            let max_cost = -200.0;
            let delta_cost = max_cost-min_cost;
            for i in 0..I_MAX {
                for j in 0..J_MAX {
                    let target = [
                        START_X + i as f64 * STEP_X,
                        START_Y + j as f64 * STEP_Y,
                        args.z_target,
                    ];
                    if target[0]*target[0] + target[1]*target[1] > radius_max_square {
                        continue;
                    } 
                    rik.reset_origin();

                    let res = rik.repeat_solve_ik(target);
                    matrix[j][i] = match res {
                        Ok(status) => {
                            let h = (status.cost_value().clamp(min_cost, max_cost) - min_cost) / delta_cost;
                            to_color(h)
                        }
                        Err(e) => {
                            f.write_all(format!("{target:?} \t {e}\n").as_bytes())
                                .unwrap();
                            err_to_color(e)
                        }
                    };
                    pb2.set_position(j as u64);
                }
                pb.set_position(i as u64);
                if i % 10 == 0 {
                    // real time plotting
                    chart
                        .draw_series(
                            matrix
                                .iter()
                                .zip(0..)
                                .flat_map(|(l, y)| l.iter().zip(0..).map(move |(v, x)| (x, y, v)))
                                .map(|(x, y, color)| {
                                    Rectangle::new([(x, y), (x + 1, y + 1)], color.filled())
                                }),
                        )
                        .unwrap();
                    let _ = root.present();
                }
            }
        }
        ScanMode::Motion => {
            let min_step = 3;
            let max_step = 10;
            let delta_step = max_step-min_step;
            for i in 0..I_MAX {
                for j in 0..J_MAX {
                    let target = [
                        START_X + i as f64 * STEP_X,
                        START_Y + j as f64 * STEP_Y,
                        args.z_target,
                    ];
                    let rad_dist = target[0]*target[0] + target[1]*target[1];
                    if rad_dist > radius_max_square {
                        continue;
                    } 
                    if rad_dist < 0.2*0.2 {
                        continue;
                    }
                    rik.reset_origin();
                    shapes[0].0.translation = Translation3::new(target[0], target[1], target[2]);
                    let compound = Compound::new(shapes.clone());
                    rik.planner.obstacles = compound;
                    let grip = rik.grip(target);
                    matrix[j][i] = match grip {
                        Ok((q1, q2, _res)) => {
                            // f.write_all(format!("Success {target:?} {_res:?}\n").as_bytes()).unwrap();
                            let h = ((q1.len()+q2.len()).clamp(min_step, max_step) - min_step) as f64 / delta_step as f64;
                            to_color(h)
                        }
                        Err(e) => {
                            f.write_all(format!("{target:?} \t {e}\n").as_bytes())
                                .unwrap();
                            err_to_color(e)
                        }
                    };
                    pb2.set_position(j as u64);
                }
                pb.set_position(i as u64);
                if i % 5 == 0 {
                    // real time plotting
                    chart
                        .draw_series(
                            matrix
                                .iter()
                                .zip(0..)
                                .flat_map(|(l, y)| l.iter().zip(0..).map(move |(v, x)| (x, y, v)))
                                .map(|(x, y, color)| {
                                    Rectangle::new([(x, y), (x + 1, y + 1)], color.filled())
                                }),
                        )
                        .unwrap();
                    let _ = root.present();
                }
            }
        }
    }

    let elapsed = time::Instant::now() - t1;
    pb.finish_and_clear();
    let _ = m.clear();
    println!("\rTotal time elapsed = {elapsed:.3?}");
    /* GRAPH */

    chart
        .draw_series(
            matrix
                .iter()
                .zip(0..)
                .flat_map(|(l, y)| l.iter().zip(0..).map(move |(v, x)| (x, y, v)))
                .map(|(x, y, color)| Rectangle::new([(x, y), (x + 1, y + 1)], color.filled())),
        )
        .unwrap();
    let _ = root.present();
}

// Cyan: Solver error on inverse kinematics (IK failed NotFiniteComputation)
// White: Ik solved but not satisfying (IK, cost higher then threshold)
// Black: Collision with bottle at start (Collision error: ... (Start))
// Magenta: Collision with bottle at End (Collision error: ... (Goal))
// Red: path planing failed ()
// between  green and red: all good, the greener the more direct the path

// To remove specific errors from log regex : ^.*(threshold).*\n?
