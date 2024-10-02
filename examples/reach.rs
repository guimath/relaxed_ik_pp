use clap::Parser;
use nalgebra::{Point2, Point3};
use relaxed_ik_lib::relaxed_ik::RelaxedIK;
use relaxed_ik_lib::utils::config_parser::Config;
use serde::Serialize;
use std::{convert::TryInto, path::PathBuf, sync::Arc, fs};
use urdf_rs::Vec3;
use urdf_viz::Viewer;
use urdf_viz::{Action, Key, WindowEvent};

#[derive(clap::ValueEnum, Clone, Default, Debug, Serialize)]
#[serde(rename_all = "kebab-case")]
enum VisMode {
    /// Just show the scene with robot and obstacle
    #[default]
    VisualOnly,
    /// Full simulation (movable target, ik & motion)
    Full,

}

/// args
#[derive(Parser)]
struct Cli {
    /// Specify target position
    #[arg(short, long, value_names=&["X", "Y", "Z"], num_args = 3, default_values_t=&[0.6, -0.5 ,0.3],  allow_hyphen_values=true)]
    target: Vec<f64>,
    /// Specify path to the settings file.
    #[arg(
        short,
        long,
        default_value = "configs/ur5.toml",
        value_name = "FILE PATH"
    )]
    settings: PathBuf,
    /// Visualization mode
    #[clap(short, long, default_value_t, value_enum)]
    mode: VisMode,
    /// Specify Camera position
    #[arg(short, long, value_names=&["PITCH", "YAW", "DIST"], num_args = 3, default_values_t=&[0.785, -0.785 ,2.5],  allow_hyphen_values=true)]
    camera_position: Vec<f32>,
    /// For motion only : show interpolated movement
    #[arg(short, long, default_value_t = false)]
    full_move: bool,
    /// For motion only : duration of move interpolate
    #[arg(short, long, default_value_t = 5.0, value_name = "SECONDS")]
    duration: f64,
}

const HOW_TO_USE_IK_STR: &str = r#"[Target movement]
Up/Down: x change
Left/Right: y change
h/l: z change
+/-: Modify increment

[actions]
r:  Reset robot to default
c:  ik compute from current pose
p:  motion compute from current pose

[toggles]
"#;



fn get_motion(rik: &mut RelaxedIK, target:[f64 ; 3], duration:f64) -> Result<Vec<Vec<f64>>, openrr_planner::Error>{
    let (mut q1, mut q2, _) = rik.grip(target)?;
    q1.reverse();
    q2.reverse();
    let grip_plan: Vec<Vec<f64>> =
        openrr_planner::interpolate(&q1.clone(), duration, 0.01)
            .unwrap()
            .into_iter()
            .map(|point| point.position)
            .collect();
    let mut grip_plan2: Vec<Vec<f64>> = openrr_planner::interpolate(&q2.clone(), 1.0, 0.01)
        .unwrap()
        .into_iter()
        .map(|point| point.position)
        .collect();
    grip_plan2.extend(grip_plan);
    Ok(grip_plan2)
}

fn get_ik(rik: &mut RelaxedIK, target:[f64 ; 3], with_reset:bool, with_approach_dist:bool)-> Vec<f64> {
    if with_reset {
        rik.reset_origin();
    }
    if with_approach_dist {
        rik.vars.robot.arms[0].displacements[rik.last_joint_num][2] =
            rik.gripper_length + rik.config.approach_dist;
    }
    let _res = rik.repeat_solve_ik(target); //x: -0.0012 y: -0.1129 z:0.0596
    // println!(" {res:?}");
    // let (pos, _quat) = rik.get_ee_pos();
    // println!(
    //     "x: {:.4} y: {:.4} z:{:.4}",
    //     target[0] - pos[0],
    //     target[1] - pos[1],
    //     target[2] - pos[2]
    // );
    rik.vars.robot.arms[0].displacements[rik.last_joint_num][2] = rik.gripper_length;
    rik.vars.xopt.clone()
}
fn main() {
    env_logger::init();
    let args = Cli::parse();
    let target = [args.target[0], args.target[1], args.target[2]];
    // let settings = args.settings.unwrap_or(PathBuf::from("configs/ur5_grip.yaml"));
    let conf = Config::from_settings_file(args.settings.clone());
    // rik init
    let mut rik = RelaxedIK::new(args.settings.to_str().unwrap());
    // urdf viz
    let (mut viewer, mut window) = Viewer::new("Example of grip");
    let mut cam = viewer.arc_ball.clone();
    cam.set_pitch(args.camera_position[0]);
    cam.set_yaw(args.camera_position[1]);
    cam.set_dist(args.camera_position[2]);
    viewer.arc_ball = cam.clone();
    viewer.add_axis_cylinders(&mut window, "test", 0.2);
    // ROBOT
    let description: urdf_rs::Robot =
        urdf_rs::read_file(rik.config.urdf_paths.robot.clone()).expect("robot URDF file not found");
    let robot: Arc<k::Chain<f64>> = Arc::new(k::Chain::from(description.clone()));
    viewer.add_robot(&mut window, &description, &conf.package_paths);
    let robot_viz = &robot;
    robot_viz.update_transforms();
    viewer.update(robot_viz);
    // OBSTACLE
    let new_obst = conf
        .urdf_paths.obstacle
        .expect("No obstacles file in config, add obstacles: FILE_PATH to config")
        .clone();
    let mut obstacle_description = urdf_rs::read_file(new_obst.clone()).unwrap();
    obstacle_description.links[0].visual[0].origin.xyz = Vec3(target);
    obstacle_description.links[0].collision[0].origin.xyz = Vec3(target);
    viewer.add_robot(&mut window, &obstacle_description, &Default::default());
    // Text options
    let menu_pose = Point2::new(10.0f32, 10.0);
    let menu_color = Point3::new(1.0f32, 1.0, 1.0);
    let menu_font = 50.0f32;
    let info_color = Point3::new(0.0f32, 1.0, 0.0);
    let info_font = 40.0;
    match args.mode {
        VisMode::VisualOnly => {
            let q = rik.vars.xopt.clone();
            robot.set_joint_positions_clamped(&q);
            let folder = args.settings.file_stem().unwrap().to_str().unwrap();
            let cam_poses = [
                (0.785, -0.785, args.camera_position[2]),
                (0.785, 0.785, args.camera_position[2]),
                (0.0, 0.0, args.camera_position[2]),
            ];
            let mut view_num = 0;
            while window.render_with_camera(&mut viewer.arc_ball) {
                viewer.update(robot_viz);
                for event in window.events().iter() {
                    if let WindowEvent::Key(code, Action::Press, _mods) = event.value {
                        match code {
                            Key::V => {
                                // Change view
                                view_num = (view_num + 1) % (cam_poses.len());
                                cam.set_pitch(cam_poses[view_num].0);
                                cam.set_yaw(cam_poses[view_num].1);
                                cam.set_dist(cam_poses[view_num].2);
                                viewer.arc_ball = cam.clone();
                            }
                            Key::C => {
                                // Capture screen
                                let screen = window.snap_image();
                                let pic_file =
                                    PathBuf::from(format!("ex_out/{folder}/view{view_num:}.png"));
                                let _ = fs::create_dir_all(pic_file.parent().unwrap());
                                let res = screen.save(pic_file);
                                if res.is_err() {
                                    println!("Error when saving screen : {res:?}")
                                }
                            }
                            _ => (),
                        }
                    }
                }
            }
        }

        VisMode::Full => {
            // Wether to display control info menu
            let mut show_menu = true;
            // Wether to compute ik with every move 
            let mut live_compute = true;
            // Wether to reset between each ik (except for manual compute)
            let mut with_reset = true;
            // wether to compute ik with approach dist
            let mut with_approach_dist = false;
            // target increment
            let mut incr = 0.1;


            let mut cur_target = [-100.0f64; 3];
            let mut next_target = target;
            let mut plans: Vec<Vec<f64>> = Vec::new();
            let mut last_gripper_pose = [0.0f64; 3];

            while window.render_with_camera(&mut viewer.arc_ball) {
                if cur_target != next_target {
                    // changed target
                    if live_compute {
                        plans.push(get_ik(&mut rik,next_target, with_reset, with_approach_dist));
                        cur_target = next_target;
                    }
                    // moving obstacle
                    viewer.remove_robot(&mut window, &obstacle_description);
                    obstacle_description.links[0].visual[0].origin.xyz = Vec3(next_target);
                    obstacle_description.links[0].collision[0].origin.xyz = Vec3(next_target);
                    viewer.add_robot(&mut window, &obstacle_description, &Default::default());
                }
                
                if !plans.is_empty() {
                    let plan = plans.pop().unwrap();
                    robot.set_joint_positions_clamped(&plan);
                    viewer.update(robot_viz);
                    let pose = rik
                        .vars
                        .robot
                        .get_ee_pos_and_quat_immutable(&plan);
                    last_gripper_pose = pose[0].0.as_slice().try_into().unwrap();
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }

                if show_menu {
                    let lc_status =  if live_compute {"on"} else {"off"};
                    let reset_status =  if with_reset {"on"} else {"off"};
                    let pre_grasp_status =  if with_approach_dist {"on"} else {"off"};
                    let how_to_use = 
                        HOW_TO_USE_IK_STR.to_string() +
                        format!("t: live compute ({lc_status})\nw: reset between ik ({reset_status})\ng: pre-grasp ik ({pre_grasp_status})\nm: menu (on)").as_str();

                    let info_pose = Point2::new(window.width() as f32 * 2.0 - 600.0, 10.0);
                    viewer.draw_text(
                        &mut window,
                        how_to_use.as_str(),
                        menu_font,
                        &menu_pose,
                        &menu_color,
                    );
                    viewer.draw_text(
                        &mut window,
                        format!("target : {:.3} {:.3} {:.3}\nDelta : {:.3} {:.3} {:.3}",
                            next_target[0], next_target[1], next_target[2],
                            last_gripper_pose[0] - next_target[0], last_gripper_pose[1] - next_target[1], last_gripper_pose[2] - next_target[2],
                        )
                            .as_str(),
                        info_font,
                        &info_pose,
                        &info_color,
                    );
                }
                
                for event in window.events().iter() {
                    if let WindowEvent::Key(code, Action::Press, _mods) = event.value {
                        match code {
                            Key::Up => next_target[0] -= incr,
                            Key::Down => next_target[0] += incr,
                            Key::Left => next_target[1] -= incr,
                            Key::Right => next_target[1] += incr,
                            Key::H => next_target[2] += incr,
                            Key::L => next_target[2] -= incr,
                            Key::Add => incr += incr / 10.0,
                            Key::Subtract => incr -= incr / 10.0,
                            Key::P => {
                                match get_motion(&mut rik,next_target, args.duration) {
                                    Ok(plan) => {
                                        plans.extend(plan);
                                        cur_target = next_target;
                                    }
                                    Err(e) => println!("Error when calc motion {e}")
                                }
                                
                            }
                            Key::C => {
                                plans.push(get_ik(&mut rik,next_target, false, with_approach_dist));
                                cur_target = next_target;
                            }
                            Key::R => {
                                rik.reset_origin();
                                plans.push(rik.vars.xopt.clone());
                            }
                            Key::T => live_compute = !live_compute,
                            Key::M => show_menu = !show_menu,
                            Key::W => with_reset = !with_reset,
                            Key::G => with_approach_dist = !with_approach_dist,
                            a => println!("{a:?}"),
                        }
                    }
                }
            }
        }
    }
}
