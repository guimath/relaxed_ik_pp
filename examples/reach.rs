use nalgebra::{Point2,Point3};
use relaxed_ik_lib::relaxed_ik_wrapper::RelaxedWrapper;
use relaxed_ik_lib::utils::config_parser::Config;
use serde::Serialize;
use urdf_rs::Vec3;
use std::path::PathBuf;
use std::sync::Arc;
use std::time;

use urdf_viz::Viewer;
use urdf_viz::{Action, Key, WindowEvent};

use clap::Parser;

#[derive(clap::ValueEnum, Clone, Default, Debug, Serialize,)]
#[serde(rename_all = "kebab-case")]
enum VisMode {
    /// Just show the scene with robot and obstacle
    #[default]
    VisualOnly,
    /// Show Ik results (movable target)
    IkOnly,
    /// Show motion 
    Motion,
}

/// args
#[derive(Parser)]
struct Cli {
    /// Specify target position
    #[arg(short, long, value_names=&["X", "Y", "Z"], num_args = 3, default_values_t=&[0.6, -0.5 ,0.3],  allow_hyphen_values=true)]
    target: Vec<f64>,
    /// Specify path to the settings file. 
    #[arg(short, long, default_value = "configs/ur5_grip.yaml", value_name="FILE PATH")]
    settings: PathBuf,
    /// Visualization mode 
    #[clap(short, long, default_value_t, value_enum)]
    mode: VisMode,
    /// For motion only : show interpolated movement
    #[arg(short, long, default_value_t = false)]
    full_move: bool,
    /// For motion only : duration of move interpolate
    #[arg(short, long, default_value_t = 5.0, value_name="SECONDS")]
    duration: f64,
}


const HOW_TO_USE_IK_STR: &str = r#"Up/Down: x change
Left/Right: y change
h/l: z change
+/-: Modify increment
c:  Manual recompute
r:  Reset robot to default
t:  toggle live recompute
m: toggle show menu
"#;

const HOW_TO_USE_MOTION_STR: &str = r#"s: start/stop motion
m: toggle show menu
"#;


fn main() {
    env_logger::init();
    let args = Cli::parse();
    let target = [args.target[0], args.target[1], args.target[2]];
    // let settings = args.settings.unwrap_or(PathBuf::from("configs/ur5_grip.yaml"));
    let conf =     Config::from_settings_file(args.settings.clone());
    // rik init
    let mut rik = RelaxedWrapper::new(args.settings.to_str().unwrap());
    // urdf viz 
    let (mut viewer, mut window) = Viewer::new("Example of grip");
    viewer.add_axis_cylinders(&mut window, "test", 0.2);
    // ROBOT
    let description : urdf_rs::Robot = urdf_rs::read_file(rik.config.robot_urdf_path.clone()).expect("robot URDF file not found");
    let robot: Arc<k::Chain<f64>> = Arc::new(k::Chain::from(description.clone()));
    viewer.add_robot(&mut window, &description, &Default::default(),);
    let robot_viz  = &robot;
    robot_viz.update_transforms();
    viewer.update(robot_viz);
    // OBSTACLE
    let new_obst = conf.obstacles_urdf_path.unwrap().clone();
    let mut obstacle_description = urdf_rs::read_file(new_obst.clone()).unwrap();
    obstacle_description.links[0].visual[0].origin.xyz = Vec3(target);
    obstacle_description.links[0].collision[0].origin.xyz = Vec3(target);
    viewer.add_robot(&mut window, &obstacle_description,&Default::default());
    // Text options
    let menu_pose  = Point2::new(10.0f32,10.0);
    let menu_color = Point3::new(1.0f32,1.0, 1.0);
    let menu_font = 50.0f32;
    let info_pose  = Point2::new(window.width()as f32-40.0,10.0);
    let info_color = Point3::new(0.0f32,1.0, 0.0);
    let info_font = 40.0;
    
    match args.mode {
        VisMode::VisualOnly => {
            let q = rik.vars.xopt.clone();
            robot.set_joint_positions_clamped(&q);
            while window.render_with_camera(&mut viewer.arc_ball) {
                viewer.update(robot_viz);
            }
        }
        
        VisMode::IkOnly =>{
            println!("{:?}", window.width());
            let mut show_menu = true;
            let mut incr = 0.1;
            let mut live_compute = true;
            let  mut cur_target = [-100.0f64; 3]; 
            let mut next_target = target.clone();
            rik.vars.robot.arms[0].displacements[rik.last_joint_num][2] = rik.gripper_length + rik.config.approach_dist;
            while window.render_with_camera(&mut viewer.arc_ball) {
                if cur_target != next_target {
                    if live_compute {
                        let res = rik.solve_ik(next_target);//x: -0.0012 y: -0.1129 z:0.0596
                        println!(" {res:?}");
                        let (pos, _quat) = rik.get_ee_pos();
                        println!("x: {:.4} y: {:.4} z:{:.4}", target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]);
                        robot.set_joint_positions_clamped(&rik.vars.xopt.clone());
                        cur_target = next_target;
                    }
                    
                    viewer.remove_robot(&mut window, &obstacle_description);
                    obstacle_description.links[0].visual[0].origin.xyz = Vec3(next_target);
                    obstacle_description.links[0].collision[0].origin.xyz = Vec3(next_target);
                    viewer.add_robot(&mut window,&obstacle_description,&Default::default());
                }
                if show_menu {
                    viewer.draw_text(&mut window,HOW_TO_USE_IK_STR, menu_font,&menu_pose, &menu_color);
                }
                viewer.draw_text(
                    &mut window,
                    format!("target : {:.3} {:.3} {:.3}", next_target[0], next_target[1], next_target[2]).as_str(),
                    info_font,
                    &info_pose, &info_color
                );

                viewer.update(robot_viz);
                for event in window.events().iter() {
                    if let WindowEvent::Key(code, Action::Press, _mods) = event.value {
                        match code {
                            Key::Up    => next_target[0] = next_target[0]-incr,
                            Key::Down  => next_target[0] = next_target[0]+incr,
                            Key::Left  => next_target[1] = next_target[1]-incr,
                            Key::Right => next_target[1] = next_target[1]+incr,
                            Key::H     => next_target[2] = next_target[2]+incr,
                            Key::L     => next_target[2] = next_target[2]-incr,
                            Key::Add      => incr += incr/10.0,
                            Key::Subtract => incr -= incr/10.0,
                            Key::C     => cur_target[0] = cur_target[0] +1.0,// just to trigger recompute
                            Key::R     => {
                                rik.reset_origin();
                                robot.set_joint_positions_clamped(&rik.vars.xopt.clone());
                            }
                            Key::T => live_compute = !live_compute,
                            Key::M => show_menu = !show_menu,
                            a => println!("{a:?}"),
                        }
                    }
                }                      
            }
        }

        VisMode::Motion => {
            let mut show_menu = true;
            let t1 = time::Instant::now();
            let (q1, q2) = match rik.grip(target) {
                Ok((x1, x2, _)) => (x1, x2),
                Err(e) => {
                    println!("error {e:}"); 
                    (vec![conf.starting_config.clone(), conf.starting_config.clone()], 
                    vec![conf.starting_config.clone(), conf.starting_config.clone()])
                }, // To show env even if failed 
            };
        
            let elapsed = time::Instant::now() - t1;
            println!("Time elapsed = {elapsed:.3?}");
        
            let mut q = q1.clone(); q.extend(q2.clone());
            let mut extra_wait = 100usize;
        
            if args.full_move { // -> interpolate
                let mut grip_plan: Vec<Vec<f64>> = openrr_planner::interpolate(&q1.clone(), args.duration, 0.01)
                    .unwrap()
                    .into_iter()
                    .map(|point| point.position)
                    .collect();
                let grip_plan2: Vec<Vec<f64>> = openrr_planner::interpolate(&q2.clone(), 1.0, 0.01)
                    .unwrap()
                    .into_iter()
                    .map(|point| point.position)
                    .collect();
                grip_plan.extend(grip_plan2);
                q = grip_plan.clone();
                extra_wait = 1; 
            }
        
            let mut incr = 1;
            let tot = q.len();
            let (mut i, mut j) = (0usize, 0usize);
            while window.render_with_camera(&mut viewer.arc_ball) {
                if show_menu {
                    viewer.draw_text(&mut window,HOW_TO_USE_MOTION_STR, menu_font,&menu_pose, &menu_color);
                }
                robot.set_joint_positions_clamped(&q[i]);
                viewer.update(robot_viz);
                j = (j+incr)%extra_wait;
                if j == 0 {i = (i+incr)%tot;}
                std::thread::sleep(std::time::Duration::from_millis(10));
                for event in window.events().iter() {
                    if let WindowEvent::Key(code, Action::Press, _mods) = event.value {
                        match code {
                            Key::S => {
                                incr = (incr + 1)%2;
                                println!("Current pos : {:?}",q[i]);
                            }
                            Key::M => {
                                show_menu = !show_menu;
                            }
                            _ => (),
                        }
                    }
                }                            
            }
        }
    }
}
