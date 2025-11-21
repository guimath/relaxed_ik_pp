use crate::{
    spacetime::arm::{JointLimits, RevoluteArm},
    utils::structs::*,
};

#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<RevoluteArm>,
    pub num_chains: usize,
    pub num_dof: usize,
    pub chain_lengths: Vec<usize>,
    pub joint_limits: Vec<JointLimits>,
}

impl Robot {
    pub fn from_urdf(urdf: &str, base_links: &[String], ee_links: &[String]) -> Self {
        // let chain = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
        let description: urdf_rs::Robot = urdf_rs::read_from_string(urdf).unwrap();
        let chain: k::Chain<f64> = k::Chain::from(description.clone());
        let mut arms: Vec<RevoluteArm> = Vec::new();
        let num_chains = base_links.len();
        let mut num_dof = 0;
        let mut chain_lengths: Vec<usize> = Vec::new();
        let mut joint_limits = Vec::new();

        for i in 0..num_chains {
            let base_link = chain.find_link(base_links[i].as_str()).unwrap_or_else(|| {
                panic!(
                    "Base link \"{}\" was not found in robot urdf",
                    base_links[i]
                )
            });
            let ee_link = chain.find_link(ee_links[i].as_str()).unwrap_or_else(|| {
                panic!(
                    "End effector link \"{}\" was not found in robot urdf",
                    ee_links[i]
                )
            });
            let serial_chain = k::SerialChain::from_end_to_root(ee_link, base_link);
            let arm = RevoluteArm::from_chain(serial_chain);
            num_dof += arm.num_dof;
            chain_lengths.push(arm.num_dof);
            joint_limits.extend(arm.joint_limits.clone());
            arms.push(arm);
        }

        Robot {
            arms,
            num_chains,
            num_dof,
            chain_lengths,
            joint_limits,
        }
    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> Vec<Pose> {
        let mut out: Vec<Pose> = Vec::new();
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out.push(self.arms[i].get_frames_immutable(&x[l..r]));
            l = r;
        }
        out
    }

    pub fn get_manipulability_with_frame(&self, x: &[f64], frame: &[Pose]) -> f64 {
        let mut out = 0.0;
        let mut l = 0;
        let mut r = 0;
        for (i, frame_i) in frame.iter().enumerate().take(self.num_chains) {
            // for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out += self.arms[i].get_manipulability_with_frame(&x[l..r], &frame_i.0, &frame_i.1);
            l = r;
        }
        out
    }

    pub fn get_ee_pos_and_quat_immutable(
        &self,
        x: &[f64],
    ) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out.push(self.arms[i].get_ee_pos_and_quat_immutable(&x[l..r]));
            l = r;
        }
        out
    }
}
