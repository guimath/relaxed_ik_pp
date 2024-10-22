use crate::spacetime::arm::RevoluteArm;
use nalgebra;
use nalgebra::{Vector3, UnitQuaternion};
use urdf_rs;

#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<RevoluteArm>,
    pub num_chains: usize,
    pub num_dof: usize,
    pub chain_lengths: Vec<usize>,
    pub upper_joint_limits: Vec<f64>,
    pub lower_joint_limits: Vec<f64>,
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
        let mut upper_joint_limits: Vec<f64> = Vec::new();
        let mut lower_joint_limits: Vec<f64> = Vec::new();

        for i in 0..num_chains {
            let base_link = chain.find_link(base_links[i].as_str()).expect(format!("Base link \"{}\" was not found in robot urdf", base_links[i]).as_str());
            let ee_link = chain.find_link(ee_links[i].as_str()).expect(format!("End effector link \"{}\" was not found in robot urdf", ee_links[i]).as_str());
            let serial_chain = k::SerialChain::from_end_to_root(ee_link, base_link);
            let arm= RevoluteArm::from_chain(serial_chain);
            num_dof += arm.num_dof;
            chain_lengths.push(arm.num_dof);
            upper_joint_limits.extend(arm.upper_joint_limits.clone());
            lower_joint_limits.extend(arm.lower_joint_limits.clone());
            arms.push(arm);
        }

        Robot {
            arms,
            num_chains,
            num_dof,
            chain_lengths,
            upper_joint_limits,
            lower_joint_limits
        }
    }

    pub fn get_frames_immutable(
        &self,
        x: &[f64],
    ) -> Vec<(
        Vec<nalgebra::Vector3<f64>>,
        Vec<nalgebra::UnitQuaternion<f64>>,
    )> {
        let mut out: Vec<(
            Vec<nalgebra::Vector3<f64>>,
            Vec<nalgebra::UnitQuaternion<f64>>,
        )> = Vec::new();
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out.push(self.arms[i].get_frames_immutable(&x[l..r]));
            l = r;
        }
        out
    }

    pub fn get_manipulability_with_frame(&self, x: &[f64], frame:&[(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)]) -> f64{
        let mut out = 0.0;
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out += self.arms[i].get_manipulability_with_frame(&x[l..r], &frame[i].0, &frame[i].1);
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
