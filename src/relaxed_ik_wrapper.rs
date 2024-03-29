
use pyo3::prelude::*;
use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective_master::ObjectiveMaster;
use crate::groove::groove::OptimizationEngineOpen;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};

#[pyclass]
pub struct RelaxedWrapper {
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen,
    pub init_q : Vec<f64>,
}

#[pymethods]
impl RelaxedWrapper {
    #[new]
    pub fn new(path_to_setting: &str) -> Self {
        println!("RelaxedIK is using below setting file {}", path_to_setting);

        let vars = RelaxedIKVars::from_local_settings(path_to_setting);
        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths);

        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());
        let init_q = vars.init_state.clone();
        Self{vars, om, groove, init_q}
    }

    pub fn solve_position(&mut self, pos_goals:[f64; 3], quat_goals:[f64; 4], tolerance:[f64; 6]) -> (Vec<f64>, Option<f64>){
        for i in 0..self.vars.robot.num_chains  {//TODO use slices
            self.vars.goal_positions[i] = Vector3::new(pos_goals[3*i], pos_goals[3*i+1], pos_goals[3*i+2]);
            let tmp_q = Quaternion::new(quat_goals[4*i+3], quat_goals[4*i], quat_goals[4*i+1], quat_goals[4*i+2]);
            self.vars.goal_quats[i] =  UnitQuaternion::from_quaternion(tmp_q);
            self.vars.tolerances[i] = Vector6::new( tolerance[6*i], tolerance[6*i+1], tolerance[6*i+2],
                tolerance[6*i+3], tolerance[6*i+4], tolerance[6*i+5])
        }
                        
        self.solve()
    }   

    pub fn solve(&mut self) -> (Vec<f64>, Option<f64>) {
        let mut out_x = self.vars.xopt.clone();
        let res = self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);
        let _frames = self.vars.robot.get_frames_immutable(&out_x);
    
        match res{
            Ok(status)=> {
                self.vars.update(out_x.clone());  
                (out_x, Some(status.cost_value()))
            },
            Err(_) => {
                println!("No valid solution found! Returning previous solution: {:?}. End effector position goals: {:?}", self.vars.xopt, self.vars.goal_positions);
                (self.vars.xopt.clone(), None)
            }
        }
    }

    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset( x.clone());
    }

    pub fn reset_origin(&mut self){
        self.vars.reset(self.init_q.clone());
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