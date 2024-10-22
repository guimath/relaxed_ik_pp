use crate::utils::config_parser::Config;
use crate::Error;
use crate::errors::point_from_str;
use nalgebra::{Isometry, Vector3};
use ncollide3d::shape::{Compound, Cuboid, ShapeHandle};
use openrr_planner::FromUrdf;
use std::sync::Arc;
pub struct Planner
// where
//     T: RealField + Copy + k::SubsetOf<f64>,
{
    pub obstacles: Compound<f64>,
    pub planner: openrr_planner::JointPathPlannerWithIk<
        f64,
        openrr_planner::RandomInitializeIkSolver<f64, k::JacobianIkSolver<f64>>,
    >,
    pub using_joint_names: Vec<String>,
}

impl Planner {
    pub fn from_config(config: Config) -> Self {
        let description =
            urdf_rs::read_file(config.urdf_paths.robot).expect("robot URDF file not found");
        let robot: Arc<k::Chain<f64>> = Arc::new(k::Chain::from(description.clone()));
        let obstacles = match config.urdf_paths.obstacle {
            Some(file_path) => {
                let urdf_obstacles =
                    urdf_rs::utils::read_urdf_or_xacro(file_path).expect("obstacle file not found");
                Compound::from_urdf_robot(&urdf_obstacles)
            }
            None => {
                log::warn!("No obstacles file given. Default to 0.03 0.03 0.8 box");
                Compound::new(vec![(
                    Isometry::identity(),
                    ShapeHandle::new(Cuboid::new(Vector3::new(0.03, 0.03, 0.8))),
                )])
            }
        };

        let planner = openrr_planner::JointPathPlannerBuilder::from_urdf_robot_with_base_dir(description.clone(), None)
            .collision_check_margin(0.01f64)
            .reference_robot(robot.clone())
            .step_length(0.05)
            .finalize()
            .unwrap();

        let solver = openrr_planner::JacobianIkSolver::default();
        let solver = openrr_planner::RandomInitializeIkSolver::new(solver, 100);
        // Create path planner with IK solver
        let planner = openrr_planner::JointPathPlannerWithIk::new(planner, solver);
        Self {
            obstacles,
            planner,
            using_joint_names: config.links.used_joints,
        }
    }


    
    pub fn get_motion(
        &mut self,
        x_start: Vec<f64>,
        x_goal: Vec<f64>,
    ) -> Result<Vec<Vec<f64>>, Error> {

        self.planner
            .plan_joints::<f64>(&self.using_joint_names, &x_start, &x_goal, &self.obstacles)
            .map_err(|e| { match e {
                openrr_planner::Error::Collision { point, collision_link_names } => {
                    Error::Collision { point:point_from_str(format!("{point:?}").as_str()), collision_link_names }
                } ,
                openrr_planner::Error::SelfCollision { point, collision_link_names } => {
                    Error::SelfCollision { point:point_from_str(format!("{point:?}").as_str()), collision_link_names }
                } ,
                openrr_planner::Error::PathPlanFail(_) => Error::PathPlanFail,
                _ => Error::Other { error: "".to_string() }
            }
            })
    }
}
