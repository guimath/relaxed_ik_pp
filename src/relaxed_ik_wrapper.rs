
use std::path::PathBuf;

use log::{info, warn};
use optimization_engine::core::SolverStatus;
use pyo3::prelude::*;
use crate::{groove::vars::RelaxedIKVars, utils::config_parser::Config};
use crate::groove::objective_master::ObjectiveMaster;
use crate::groove::groove::OptimizationEngineOpen;
use crate::motion::planner::Planner;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};

#[pyclass]
pub struct RelaxedWrapper {
    pub config: Config,
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen,
    pub init_q : Vec<f64>,
    pub planner: Planner,
    last_joint_num : usize,
    gripper_length : f64,
}

#[pymethods]
impl RelaxedWrapper {
    #[new]
    pub fn new(path_to_setting: &str) -> Self {
        info!("Using settings file {path_to_setting}");
        
        let config = Config::from_settings_file(PathBuf::from(path_to_setting)); 
        info!("Parsing successful {:?}", config.clone());
        let vars = RelaxedIKVars::from_config(config.clone());

        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths);

        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());
        let init_q = vars.init_state.clone();
        let planner = Planner::from_config(config.clone());
        let pos_goals   = [0.6f64, -0.5, 0.3];
        let quat_goals  = [0.707f64, 0.0, 0.707, 0.0];
        let tolerances  = [0.0f64; 6];
        let last_joint_num = vars.robot.arms[0].displacements.len()-1;
        let gripper_length = vars.robot.arms[0].displacements[last_joint_num][2];
        let mut a = Self{config, vars, om, groove, init_q, planner, last_joint_num, gripper_length};
        a.set_ik_params(pos_goals, quat_goals, tolerances);
        a
    }

    pub fn solve_position(&mut self, pos_goals:[f64; 3], quat_goals:[f64; 4], tolerance:[f64; 6]) -> (Vec<f64>, Option<f64>){
        
        self.set_ik_params(pos_goals, quat_goals, tolerance);
                        
        let res = self.solve_ik(pos_goals);
        match res{
            Ok(status)=> {
                self.vars.update(self.vars.xopt.clone());  
                (self.vars.xopt.clone(), Some(status.cost_value()))
            },
            Err(_) => {
                warn!("No valid solution found! Returning previous solution: {:?}. End effector position goals: {:?}", self.vars.xopt, self.vars.goal_positions);
                (self.vars.xopt.clone(), None)
            }
        }
    }   
    
    pub fn get_ee_pos(&self) -> (Vec<f64>, Vec<f64>){
        let pose = self.vars.robot.get_ee_pos_and_quat_immutable(&self.vars.xopt);
        (pose[0].0.as_slice().to_vec(), pose[0].1.as_vector().as_slice().to_vec())
    }

    pub fn get_objectives_costs(&self) -> Vec<f64> {
        self.om.get_costs(&self.vars.xopt, &self.vars)
    }
    
    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset( x.clone());    
    }

    pub fn reset_origin(&mut self){
        self.vars.reset(self.init_q.clone());

    }
}

/// pure rust
impl RelaxedWrapper {
    pub fn grip(&mut self, pos_goals:[f64; 3]) -> Result<(Vec<Vec<f64>>, SolverStatus), openrr_planner::Error>{
        let res = self._grip(pos_goals);
        self.vars.robot.arms[0].displacements[self.last_joint_num][2] = self.gripper_length;// in case first ik unsuccessful 
        res
    }

    fn _grip(&mut self, pos_goals:[f64; 3]) -> Result<(Vec<Vec<f64>>, SolverStatus), openrr_planner::Error>{
        let x_start = self.vars.xopt.clone();
        self.vars.robot.arms[0].displacements[self.last_joint_num][2] = self.gripper_length + self.config.approach_dist;
        let _res1 = self.solve_ik(pos_goals)?;
        let x_inter = self.vars.xopt.clone();
        self.vars.robot.arms[0].displacements[self.last_joint_num][2] = self.gripper_length;
        let res = self.solve_ik(pos_goals)?;
        let x_goal = self.vars.xopt.clone();
        

        let mut q = self.planner.get_motion(x_start, x_inter.clone())?;
        let  q2 = &self.planner.get_motion(x_inter, x_goal)?[1..]; // first already in q
        q.extend(q2.to_vec());
        Ok((q, res)) 
    } 

    pub fn set_ik_params(&mut self, pos_goals:[f64; 3], quat_goals:[f64; 4], tolerance:[f64; 6]){
        for i in 0..self.vars.robot.num_chains  {//TODO use slices
            self.vars.goal_positions[i] = Vector3::new(pos_goals[3*i], pos_goals[3*i+1], pos_goals[3*i+2]);
            let tmp_q = Quaternion::new(quat_goals[4*i+3], quat_goals[4*i], quat_goals[4*i+1], quat_goals[4*i+2]);
            self.vars.goal_quats[i] =  UnitQuaternion::from_quaternion(tmp_q);
            self.vars.tolerances[i] = Vector6::new( tolerance[6*i], tolerance[6*i+1], tolerance[6*i+2],
                tolerance[6*i+3], tolerance[6*i+4], tolerance[6*i+5])
        }
    }
    pub fn solve_ik(&mut self, pos_goals:[f64; 3]) -> Result<SolverStatus, openrr_planner::Error> {
        for i in 0..self.vars.robot.num_chains  {
            self.vars.goal_positions[i] = Vector3::new(pos_goals[3*i], pos_goals[3*i+1], pos_goals[3*i+2]);
        }
        let mut out_x = self.vars.xopt.clone();

        match self.groove.optimize(&mut out_x, &self.vars, &self.om, 100) {
            Ok(stat) => {
                let cost = stat.cost_value();
                if cost > self.config.cost_threshold { // TODO use custom Error
                    Err(openrr_planner::Error::ParseError(format!("ik, cost higher then threshold {:?} {}", stat.cost_value(), self.config.cost_threshold))) 
                }
                else {
                    self.vars.update(out_x.clone());
                    Ok(stat)
                }
            } 
            Err(e) => Err(openrr_planner::Error::Other{error:format!("IK failed {e:?}")})
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