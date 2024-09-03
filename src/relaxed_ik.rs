use crate::groove::groove::OptimizationEngineOpen;
use crate::groove::objective_master::ObjectiveMaster;
use crate::groove::vars::RelaxedIKVars;
use std::os::raw::{c_double, c_int};

#[repr(C)]
pub struct Opt {
    pub data: *const c_double,
    pub length: c_int,
}

pub struct RelaxedIK {
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen,
}

impl RelaxedIK {
    pub fn load_settings(path_to_setting: &str) -> Self {
        println!("RelaxedIK is using below setting file {}", path_to_setting);

        let vars = RelaxedIKVars::from_local_settings(path_to_setting);
        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths);

        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs);

        Self { vars, om, groove }
    }

    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset(x.clone());
    }

    pub fn solve(&mut self) -> Vec<f64> {
        let mut out_x = self.vars.xopt.clone();
        let res = self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);
        let _frames = self.vars.robot.get_frames_immutable(&out_x);

        if res.is_err() {
            println!("No valid solution found! Returning previous solution: {:?}. End effector position goals: {:?}", self.vars.xopt, self.vars.goal_positions);
            return self.vars.xopt.clone();
        }

        self.vars.update(out_x.clone());
        out_x
    }
}
