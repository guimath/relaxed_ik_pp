
use relaxed_ik_lib::relaxed_ik_wrapper::RelaxedWrapper;
use relaxed_ik_lib::utils::config_parser::Config;
use std::path::PathBuf;
use std::fs::File;
use std::io::prelude::*;
use std::sync::Arc;

use urdf_viz::Viewer;

use clap::Parser;

/// args
#[derive(Parser)]
struct Cli {
    /// Specify a target xyz (separated by comma)
    #[arg(short, long, use_value_delimiter = true, value_delimiter= ',')]
    target: Option<Vec<f64>>,
    /// Specify path to the settings file
    #[arg(short, long)]
    settings: Option<PathBuf>,
    /// Shows interpolated movement
    #[arg(short, long, default_value_t = false)]
    full_move: bool,
}
fn main() {
    env_logger::init();
    let args = Cli::parse();

    let target = match args.target {
        Some(v) => {
            if v.len() == 3 {[v[0], v[1], v[2]]}
            else { [0.6f64, -0.5, 0.3]}
        }
        None => [0.6f64, -0.5, 0.3],
    };
    let settings = match args.settings{
        Some(s) => s,
        None => PathBuf::from("../python/configs/ur5_grip.yaml")
    };
    let conf =     Config::from_settings_file(settings.clone());
    // dbg!(conf.clone());

    // obstacle manipulation to change position
    let new_obst = conf.obstacles_urdf_path.unwrap().clone();
    let mut obstacle_urdf = urdf_rs::read_file(new_obst.clone()).unwrap();
    obstacle_urdf.links[0].visual[0].origin.xyz.0 = target.clone();
    obstacle_urdf.links[0].collision[0].origin.xyz.0 = target.clone();
    let s = urdf_rs::write_to_string(&obstacle_urdf).unwrap();
    
    let mut old_file: Vec<u8> = vec![];
    let mut file = File::open(new_obst.clone()).unwrap();
    let _ = file.read_to_end(&mut old_file);
    let mut file = File::create(new_obst.clone()).unwrap();
    let _ = file.write_all(s.as_bytes()).unwrap();
    let _ = file.flush().unwrap();


    let mut rik = RelaxedWrapper::new(settings.to_str().unwrap());
    let q = rik.grip(target);
    // dbg!("{:?}", q);
    let grip_plan: Vec<Vec<f64>> = openrr_planner::interpolate(&q.clone(), 5.0, 0.1)
        .unwrap()
        .into_iter()
        .map(|point| point.position)
        .collect();

    // VISUALIZATION 
    let (mut viewer, mut window) = Viewer::new("Example of grip");
    // let mut packages_path: HashMap<String, String> = HashMap::new();
    // let package_path = urdf_rs::utils::rospack_find("ur_description").unwrap();
    // packages_path.insert("ur_description".to_string(), package_path);
    let description : urdf_rs::Robot = urdf_rs::read_file(rik.config.robot_urdf_path.clone()).expect("robot URDF file not found");
    let robot: Arc<k::Chain<f64>> = Arc::new(k::Chain::from(description.clone()));

    viewer.add_robot_with_base_dir(
        &mut window,
        &description,
        rik.config.robot_urdf_path.clone().parent(),
        &Default::default(),
    );
    let robot_viz  = &robot;
    robot_viz.update_transforms();
    viewer.update(robot_viz);


    /* OBSTACLES */

    let urdf_obstacles = rik.planner.obstacles_robot.clone();
    viewer.add_robot(
        &mut window,
        &urdf_obstacles,
        &Default::default(),
    );    // viewer.add_robot(&mut window, &urdf_obstacles, &Default::default());
    // resetting file
    let mut file = File::create(new_obst.clone()).unwrap();
    let _ = file.write_all(&old_file).unwrap();
    let _ = file.flush();

    let mut i = 0;
    if args.full_move {
        let tot = grip_plan.len();
        while window.render_with_camera(&mut viewer.arc_ball) {
            robot.set_joint_positions_clamped(&grip_plan[i]);
            viewer.update(robot_viz);
            // let plan = planner.
            // plan_joints::<f64>(&using_joint_names, &start_angles, &plans[i], &obstacles)
            // .unwrap();
            i = (i+1)%tot;
            std::thread::sleep(std::time::Duration::from_millis(30));
        }
    }
    else {
        let tot = q.len();
        let mut j = 0usize;
        while window.render_with_camera(&mut viewer.arc_ball) {
            robot.set_joint_positions_clamped(&q[i]);
            viewer.update(robot_viz);
            j = (j+1)%100;
            if j == 0 {i = (i+1)%tot;}
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

    }

}