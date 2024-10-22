use std::path::PathBuf;
use std::time::Instant;

use optimization_engine::core::SolverStatus;
use optimization_engine::SolverError;
// use pyo3::exceptions::PyOSError;
use crate::Error;
use crate::groove::groove::OptimizationEngineOpen;
use crate::groove::objective_master::ObjectiveMaster;
use crate::motion::planner::Planner;
use crate::{groove::vars::RelaxedIKVars, utils::config_parser::Config};
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};
pub struct RelaxedIK {
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

impl RelaxedIK {
    pub fn new(path_to_setting: &str) -> Self {
        let t1 = Instant::now();
        log::info!("Using settings file {path_to_setting}");
        
        let config = Config::from_settings_file(PathBuf::from(path_to_setting));
        log::debug!("Parsing successful {:?}", config.clone());
        let vars = RelaxedIKVars::from_config(config.clone());
        log::debug!("Robot created");

        let om = ObjectiveMaster::relaxed_ik(
            &vars.robot.chain_lengths, 
            &vars.robot.lower_joint_limits,
            &vars.robot.upper_joint_limits,
            config.objectives.clone());
            log::debug!("Objectives created");

        let groove = OptimizationEngineOpen::new(vars.robot.num_dof);
        log::debug!("Optimizer created");
        let init_q = vars.init_state.clone();
        let planner = Planner::from_config(config.clone());
        log::debug!("Planner created");
        let pos_goals = [0.6f64, -0.5, 0.3];
        let quat_goals = [0.707f64, 0.0, 0.707, 0.0];
        let tolerances = [0.0f64; 6];
        let last_joint_num = vars.robot.arms[0].num_dof;
        let gripper_length = vars.robot.arms[0].lin_offsets[last_joint_num][2];
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
        log::debug!("rik fully initialized in {dur:.3?} - minimum possible cost {min_possible_cost:.3?}");
        a
    }

    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset(x.clone());
    }

    pub fn reset_origin(&mut self) {
        self.vars.reset(self.init_q.clone());
    }
    /// Creates steps to get to given object position and grasp.
    pub fn grip(
        &mut self,
        pos_goals: [f64; 3],
    ) -> Result<(Vec<Vec<f64>>, Vec<Vec<f64>>, SolverStatus), Error> {
        let (x_start, x_inter, x_goal, res) = self._grip_two_ik(pos_goals)?;
        let q = self.planner.get_motion(x_start, x_inter.clone())?;
        let q2 = self.planner.get_motion(x_inter, x_goal)?; //[1..]; // first already in q
        Ok((q, q2, res))
    }

    pub fn grip_two_ik(        
        &mut self,
        pos_goals: [f64; 3],
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>,SolverStatus), Error> {
        let res = self._grip_two_ik(pos_goals);
        self.vars.robot.arms[0].lin_offsets[self.last_joint_num][2] = self.gripper_length; // in case first ik unsuccessful
        res
    }

    /// helper func so that if error arises, joint displacement is kept.
    fn _grip_two_ik(
        &mut self,
        pos_goals: [f64; 3],
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>,SolverStatus), Error> {
        let x_start = self.vars.xopt.clone();
        self.vars.robot.arms[0].lin_offsets[self.last_joint_num][2] =
            self.gripper_length + self.config.approach_dist;
        let _res1 = self.repeat_solve_ik(pos_goals)?;
        let x_inter = self.vars.xopt.clone();
        self.vars.robot.arms[0].lin_offsets[self.last_joint_num][2] = self.gripper_length;
        let res = self.repeat_solve_ik(pos_goals)?;
        let x_goal = self.vars.xopt.clone();
        Ok((x_start, x_inter, x_goal, res))

                                                            // q.extend(q2.to_vec());
    }

    /// Sets parameters for inverse kinematics (they will be used subsequently except if specified otherwise)
    pub fn set_ik_params(
        &mut self,
        pos_goals: [f64; 3],
        quat_goals: [f64; 4],
        tolerance: [f64; 6],
    ) {
        for i in 0..self.vars.robot.num_chains {
            self.vars.goal_positions[i] = 
                Vector3::from_row_slice(&pos_goals[3*i..3*i + 3]);
            let tmp_q = Quaternion::new(
                quat_goals[4 * i + 3],
                quat_goals[4 * i],
                quat_goals[4 * i + 1],
                quat_goals[4 * i + 2],
            );
            self.vars.goal_quats[i] = UnitQuaternion::from_quaternion(tmp_q);
            self.vars.tolerances[i] = Vector6::from_row_slice(&tolerance[6*i..6*i + 6]);
        }
    }

    pub fn repeat_solve_ik(&mut self, pos_goals: [f64; 3])-> Result<SolverStatus, Error>{
        let cost_threshold= self.min_possible_cost+10.0;//self.config.cost_threshold;
        const MAX_IK_ITER:usize = 4;
        // self.config.cost_threshold = 1000.0;
        //init vars
        for i in 0..self.vars.robot.num_chains {
            self.vars.goal_positions[i] =
                Vector3::new(pos_goals[3 * i], pos_goals[3 * i + 1], pos_goals[3 * i + 2]);
        }
        // let mut prev_cost = 200.0;

        for i in 0..(MAX_IK_ITER-1) {
            match self._optimize_ik(){
                Err(err) => {
                    return Err(err);
                }
                Ok(solve_status) => {
                    let cost = solve_status.cost_value();
                    if cost < cost_threshold {
                        log::debug!("reached {} after {} IK solved", cost, i+1);
                        // self.config.cost_threshold = prev_cost_threshold;
                        return Ok(solve_status);
                    }
                    // if (prev_cost - cost).abs() < 1.0 { // ik stuck, try resetting
                    //     self.reset_origin();
                    //     log::debug!("stuck {} (IK {})",cost, i+1)
                    //     // return Err(openrr_planner::Error::Other {error: "IK stuck".to_string()})
                    // }
                    // prev_cost = cost;
                }
            };
        }
        // self.config.cost_threshold = prev_cost_threshold;
        self._optimize_ik()
    }

    pub fn _optimize_ik (&mut self) -> Result<SolverStatus, Error> {
        let mut out_x = self.vars.xopt.clone();
        let res = self.groove.optimize(&mut out_x, &self.vars, &self.om, 200)
            .map_err(|e| {
                match e {
                    SolverError::Cost => Error::Cost,
                    SolverError::NotFiniteComputation => Error::NotFiniteComputation,
                }
            });
        if res.is_ok() {self.vars.update(out_x.clone());}
        res
    }
}
