use std::path::PathBuf;
use std::time::Instant;

use log::{debug, info, warn};
use optimization_engine::core::SolverStatus;
use pyo3::prelude::*;
// use pyo3::exceptions::PyOSError;
use crate::groove::groove::OptimizationEngineOpen;
use crate::groove::objective_master::ObjectiveMaster;
use crate::motion::planner::Planner;
use crate::{groove::vars::RelaxedIKVars, utils::config_parser::Config};
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};
#[pyclass]
pub struct RelaxedWrapper {
    pub config: Config,
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen,
    pub init_q: Vec<f64>,
    pub planner: Planner,
    pub last_joint_num: usize,
    pub gripper_length: f64,
    pub min_possible_cost: f64,
}

#[pymethods]
impl RelaxedWrapper {
    #[new]
    pub fn new(path_to_setting: &str) -> Self {
        let t1 = Instant::now();
        info!("Using settings file {path_to_setting}");

        let config = Config::from_settings_file(PathBuf::from(path_to_setting));
        debug!("Parsing successful {:?}", config.clone());
        let vars = RelaxedIKVars::from_config(config.clone());
        debug!("Robot created");

        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths);
        debug!("Objectives created");

        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs);
        debug!("Optimizer created");
        let init_q = vars.init_state.clone();
        let planner = Planner::from_config(config.clone());
        debug!("Planner created");
        let pos_goals = [0.6f64, -0.5, 0.3];
        let quat_goals = [0.707f64, 0.0, 0.707, 0.0];
        let tolerances = [0.0f64; 6];
        let last_joint_num = vars.robot.arms[0].displacements.len() - 1;
        let gripper_length = vars.robot.arms[0].displacements[last_joint_num][2];
        let min_possible_cost: f64 = - om.weight_priors.iter().sum::<f64>();
        let mut a = Self {
            config,
            vars,
            om,
            groove,
            init_q,
            planner,
            last_joint_num,
            gripper_length,
            min_possible_cost,
        };
        a.set_ik_params(pos_goals, quat_goals, tolerances);
        let dur = Instant::now() - t1; 
        debug!("rik fully initialized in {dur:.3?} - minimum possible cost {min_possible_cost:.3?}");
        a
    }

    pub fn solve_position(
        &mut self,
        pos_goals: [f64; 3],
        quat_goals: [f64; 4],
        tolerance: [f64; 6],
    ) -> (Vec<f64>, Option<f64>) {
        self.set_ik_params(pos_goals, quat_goals, tolerance);

        let res = self.solve_ik(pos_goals);
        match res {
            Ok(status) => {
                self.vars.update(self.vars.xopt.clone());
                (self.vars.xopt.clone(), Some(status.cost_value()))
            }
            Err(_) => {
                warn!("No valid solution found! Returning previous solution: {:?}. End effector position goals: {:?}", self.vars.xopt, self.vars.goal_positions);
                (self.vars.xopt.clone(), None)
            }
        }
    }

    pub fn get_ee_pos(&self) -> (Vec<f64>, Vec<f64>) {
        let pose = self
            .vars
            .robot
            .get_ee_pos_and_quat_immutable(&self.vars.xopt);
        (
            pose[0].0.as_slice().to_vec(),
            pose[0].1.as_vector().as_slice().to_vec(),
        )
    }

    pub fn get_objectives_costs(&self) -> Vec<f64> {
        self.om.get_costs(&self.vars.xopt, &self.vars)
    }

    pub fn grip_unwrap(&mut self, pos_goals: [f64; 3]) -> Vec<Vec<f64>> {
        self.grip(pos_goals).unwrap().0
    }
    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset(x.clone());
    }

    pub fn reset_origin(&mut self) {
        self.vars.reset(self.init_q.clone());
    }
}

/// pure rust
impl RelaxedWrapper {
    /// Creates steps to get to given object position and grasp.
    pub fn grip(
        &mut self,
        pos_goals: [f64; 3],
    ) -> Result<(Vec<Vec<f64>>, Vec<Vec<f64>>, SolverStatus), openrr_planner::Error> {
        let res = self._grip(pos_goals);
        self.vars.robot.arms[0].displacements[self.last_joint_num][2] = self.gripper_length; // in case first ik unsuccessful
        res
    }

    /// helper func so that if error arises, joint displacement is kept.
    fn _grip(
        &mut self,
        pos_goals: [f64; 3],
    ) -> Result<(Vec<Vec<f64>>, Vec<Vec<f64>>, SolverStatus), openrr_planner::Error> {
        let x_start = self.vars.xopt.clone();
        self.vars.robot.arms[0].displacements[self.last_joint_num][2] =
            self.gripper_length + self.config.approach_dist;
        let _res1 = self.repeat_solve_ik(pos_goals)?;
        let x_inter = self.vars.xopt.clone();
        self.vars.robot.arms[0].displacements[self.last_joint_num][2] = self.gripper_length;
        let res = self.solve_ik(pos_goals)?;
        let x_goal = self.vars.xopt.clone();

        let q = self.planner.get_motion(x_start, x_inter.clone())?;
        let q2 = self.planner.get_motion(x_inter, x_goal)?; //[1..]; // first already in q
                                                            // q.extend(q2.to_vec());
        Ok((q, q2, res))
    }

    /// Sets parameters for inverse kinematics (they will be used subsequently except if specified otherwise)
    pub fn set_ik_params(
        &mut self,
        pos_goals: [f64; 3],
        quat_goals: [f64; 4],
        tolerance: [f64; 6],
    ) {
        for i in 0..self.vars.robot.num_chains {
            //TODO use slices
            self.vars.goal_positions[i] =
                Vector3::new(pos_goals[3 * i], pos_goals[3 * i + 1], pos_goals[3 * i + 2]);
            let tmp_q = Quaternion::new(
                quat_goals[4 * i + 3],
                quat_goals[4 * i],
                quat_goals[4 * i + 1],
                quat_goals[4 * i + 2],
            );
            self.vars.goal_quats[i] = UnitQuaternion::from_quaternion(tmp_q);
            self.vars.tolerances[i] = Vector6::new(
                tolerance[6 * i],
                tolerance[6 * i + 1],
                tolerance[6 * i + 2],
                tolerance[6 * i + 3],
                tolerance[6 * i + 4],
                tolerance[6 * i + 5],
            )
        }
    }


    pub fn repeat_solve_ik(&mut self, pos_goals: [f64; 3])-> Result<SolverStatus, openrr_planner::Error>{
        let prev_cost_threshold= -260.0;//self.config.cost_threshold;
        const MAX_IK_ITER:usize = 4;
        // self.config.cost_threshold = 1000.0;
        //init vars
        for i in 0..self.vars.robot.num_chains {
            self.vars.goal_positions[i] =
                Vector3::new(pos_goals[3 * i], pos_goals[3 * i + 1], pos_goals[3 * i + 2]);
        }

        for i in 0..(MAX_IK_ITER-1) {
            match self._optimize_ik(){
                Err(err) => {
                    // self.config.cost_threshold = prev_cost_threshold;
                    return Err(err);
                }
                Ok(solve_status) => {
                    if solve_status.cost_value() < prev_cost_threshold {
                        debug!("reached {} after {} IK solved", prev_cost_threshold, i);
                        // self.config.cost_threshold = prev_cost_threshold;
                        return Ok(solve_status);
                    }
                }
            };
        }
        // self.config.cost_threshold = prev_cost_threshold;
        self._optimize_ik()
    }

    /// gets solution using relaxed ik, with threshold test
    pub fn solve_ik(&mut self, pos_goals: [f64; 3]) -> Result<SolverStatus, openrr_planner::Error> {
        for i in 0..self.vars.robot.num_chains {
            self.vars.goal_positions[i] =
                Vector3::new(pos_goals[3 * i], pos_goals[3 * i + 1], pos_goals[3 * i + 2]);
        }
        self._optimize_ik()
    }
    
    fn _optimize_ik (&mut self) -> Result<SolverStatus, openrr_planner::Error> {
        let mut out_x = self.vars.xopt.clone();
        match self.groove.optimize(&mut out_x, &self.vars, &self.om, 200) {
            Ok(stat) => {
                let cost = stat.cost_value();
                if cost > 100.0 {
                    // TODO use custom Error
                    Err(openrr_planner::Error::ParseError(format!(
                        "ik, cost higher then threshold {:?} {}",
                        stat.cost_value(),
                        100.0
                    )))
                } else {
                    self.vars.update(out_x.clone());
                    Ok(stat)
                }
            }
            Err(e) => Err(openrr_planner::Error::Other {
                error: format!("IK failed {e:?}"),
            }),
        }
    }
}

/// A Python module implemented in Rust. The name of this function must match
/// the `lib.name` setting in the `Cargo.toml`, else Python will not be able to
/// import the module.
#[pymodule]
fn relaxed_ik_lib(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<RelaxedWrapper>()?;
    Ok(())
}
