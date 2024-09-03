use crate::utils::config_parser::Config;
use log::warn;
use nalgebra::{Isometry, Vector3};
use ncollide3d::shape::{Compound, Cuboid, ShapeHandle};
use openrr_planner::FromUrdf;
use std::sync::Arc;
use urdf_rs::Robot;
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
        let description: Robot =
            urdf_rs::read_file(config.robot_urdf_path).expect("robot URDF file not found");
        let robot: Arc<k::Chain<f64>> = Arc::new(k::Chain::from(description.clone()));

        let obstacles = match config.obstacles_urdf_path {
            Some(file_path) => {
                let urdf_obstacles =
                    urdf_rs::utils::read_urdf_or_xacro(file_path).expect("obstacle file not found");
                Compound::from_urdf_robot(&urdf_obstacles)
            }
            None => {
                warn!("No obstacles file given. Default to 0.03 0.03 0.8 box");
                Compound::new(vec![(
                    Isometry::identity(),
                    ShapeHandle::new(Cuboid::new(Vector3::new(0.03, 0.03, 0.8))),
                )])
            }
        };

        // TODO why takes a lot of time
        let planner = openrr_planner::JointPathPlannerBuilder::from_urdf_robot(description.clone())
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
            using_joint_names: config.used_links,
        }
    }

    pub fn get_motion(
        &mut self,
        x_start: Vec<f64>,
        x_goal: Vec<f64>,
    ) -> Result<Vec<Vec<f64>>, openrr_planner::Error> {
        self.planner
            .plan_joints::<f64>(&self.using_joint_names, &x_start, &x_goal, &self.obstacles)
    }
}
