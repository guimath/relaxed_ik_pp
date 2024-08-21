
use openrr_planner::FromUrdf;
use ncollide3d::shape::Compound;
use relaxed_ik_lib::relaxed_ik_wrapper::RelaxedWrapper;
use relaxed_ik_lib::utils::config_parser::Config;
use std::path::PathBuf;
use std::time::Duration;
use std::{fs, time};
use nalgebra::Translation3;
use std::fs::File;
use std::io::Write;
use clap::Parser;
use plotters::prelude::*;
/// args
#[derive(Parser)]
struct Cli {
    /// Specify path to the settings file
    #[arg(short, long)]
    settings: Option<PathBuf>,
}
fn main() {
    env_logger::init();
    let args = Cli::parse();

    const TARGET_X : f64 = -1.3;
    const TARGET_Y : f64 = -1.3;
    const TARGET_Z : f64 = 0.3;

    const STEP_X : f64= 0.01;
    const STEP_Y : f64= 0.01;
    const MAX_X  : f64= 1.3;
    const MAX_Y  : f64= 1.3;

    const I_MAX  :usize= ((MAX_X - TARGET_X) / STEP_X ) as usize; 
    const J_MAX  :usize= ((MAX_Y - TARGET_Y) / STEP_Y) as usize;
    const AVG_TIME_ONE_CALC_MS:u64 = 9;
    let est_time = Duration::from_millis((I_MAX * J_MAX) as u64 * AVG_TIME_ONE_CALC_MS); 
    println!("Total tests planned {:} ({:}x{:}). Estimated time : {:.3?}",I_MAX*J_MAX, I_MAX, J_MAX, est_time);
    let settings = match args.settings{
        Some(s) => s,
        None => PathBuf::from("configs/ur5_grip.yaml")
    };
    let conf =     Config::from_settings_file(settings.clone());

    // obstacle manipulation to change position
    let new_obst = conf.obstacles_urdf_path.unwrap().clone();
    let obstacle_urdf = urdf_rs::read_file(new_obst.clone()).unwrap();
    let obstacle = Compound::from_urdf_robot(&obstacle_urdf);
    let mut shapes = obstacle.shapes().to_vec();//[0].0.translation = [TARGET_X, TARGET_Y, TARGET_Z];
    
    let _ = fs::create_dir_all("ex_out");
    let mut f = File::create("ex_out/metric_scan.log").unwrap();
    let mut rik = RelaxedWrapper::new(settings.to_str().unwrap());
    
    let t1 = time::Instant::now();
    let mut matrix = [[WHITE.to_rgba(); I_MAX]; J_MAX];
    for i in 0..I_MAX {
        for j in 0..J_MAX {
            print!(" \r{i:} {j:}  ");
            let target = [TARGET_X + i as f64 *STEP_X, TARGET_Y+ j as f64 *STEP_Y, TARGET_Z];
            shapes[0].0.translation = Translation3::new(target[0], target[1], target[2]);
            let compound = Compound::new(shapes.clone());
            rik.planner.obstacles = compound;
            rik.reset_origin();
            let grip = rik.grip(target.clone());
            matrix[j][i] = match grip {
                
                Ok((q1, q2, _res)) => {
                    let h = (150 - (10* (q1.len()+q2.len()-1)).clamp(0, 125)) as f64 /360.0;
                    HSLColor(h, 0.7, 0.45).to_rgba()
            },
                Err(e) => {
                    f.write_all(format!("{target:?} \t {e}\n").as_bytes()).unwrap();
                    match e {
                        openrr_planner::Error::ParseError{..} => {WHITE.to_rgba() },// cost to high
                        openrr_planner::Error::Other{error:_} => {CYAN.to_rgba() },// SolverError
                        openrr_planner::Error::Collision{point:p, collision_link_names:_} => {
                            if format!("{p:?}") == "Start" {BLACK.to_rgba()} // UnfeasibleTrajectoryPoint is private :(
                            else {MAGENTA.to_rgba()} // goal collision
                        },
                        _ => RED.to_rgba(), // Path planning failed

                    }
                },
            };

        }
    }
    let elapsed = time::Instant::now() - t1;
    println!("\rTotal time elapsed = {elapsed:.3?}");

    /* GRAPH */
    let root = BitMapBackend::new("ex_out/metric_scan.png", (1024, 1024)).into_drawing_area();

    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .caption(format!("Z = {TARGET_Z:}"), ("sans-serif", 40))
        .margin(5)
        .top_x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(0i32..(I_MAX as i32), (J_MAX as i32)..0i32).unwrap();
    
    chart
        .configure_mesh()
        .x_desc("x")
        .y_desc("y")
        .x_labels(10)
        .y_labels(10)
        .max_light_lines(4)
        .x_label_formatter(&|r| format!("{:.2}", TARGET_X + STEP_X * (*r as f64)))
        .y_label_formatter(&|r| format!("{:.2}", TARGET_Y + STEP_Y * (*r as f64)))
        .x_label_offset(35)
        .y_label_offset(25)
        .disable_x_mesh()
        .disable_y_mesh()
        .label_style(("sans-serif", 20))
        .draw().unwrap();
    
    chart.draw_series(
        matrix
            .iter()
            .zip(0..)
            .flat_map(|(l, y)| l.iter().zip(0..).map(move |(v, x)| (x, y, v)))
            .map(|(x, y, color)| {
                Rectangle::new(
                    [(x, y), (x + 1, y + 1)],
                    color.filled()
                ) 
            })
    ).unwrap();
    let _ = root.present();
}

// Cyan: Solver error on inverse kinematics (IK failed NotFiniteComputation)
// White: Ik solved but not satisfying (IK, cost higher then threshold)
// Black: Collision with bottle at start (Collision error: ... (Start)) 
// Magenta: Collision with bottle at End (Collision error: ... (Goal)) 
// Red: path planing failed ()
// between  green and red: all good, the greener the more direct the path 

// To remove specific errors from log regex : ^.*(threshold).*\n?