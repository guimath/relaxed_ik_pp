use crate::{
    core::{loss::LossFunction, vars},
    utils::structs::*,
};

use nalgebra::Vector3;
use parry3d_f64::{query, shape};

pub trait ObjectiveTrait {
    fn call(
        &self,
        x: &[f64],               // joint values
        v: &vars::RelaxedIKVars, // general config variables (like target etx)
        frames: &[Pose],         // all frames poses
    ) -> f64; // returns loss value

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &[SinglePose]) -> f64;
    fn gradient(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &[Pose]) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call(x, v, frames);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000000001;
            let frames_h = v.robot.get_frames_immutable(x_h.as_slice());
            let f_h = self.call(x_h.as_slice(), v, &frames_h);
            grad.push((-f_0 + f_h) / 0.000000001);
        }

        (f_0, grad)
    }
    fn gradient_lite(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        ee_poses: &[SinglePose],
    ) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call_lite(x, v, ee_poses);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.0000001;
            let ee_poses_h = v.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
            let f_h = self.call_lite(x_h.as_slice(), v, &ee_poses_h);
            grad.push((-f_0 + f_h) / 0.0000001);
        }

        (f_0, grad)
    }
    fn gradient_type(&self) -> usize {
        1
    } // manual diff = 0, finite diff = 1
}

pub struct CardinalDirectionObjective<F: LossFunction> {
    pub arm_idx: usize,
    pub target_direction: [f64; 3], // Unit vector for the desired direction
    pub loss_fn: F,
}

impl<F: LossFunction> ObjectiveTrait for CardinalDirectionObjective<F> {
    #[inline]
    fn call(&self, _x: &[f64], _v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let ee_pos = frames[self.arm_idx].0[last_elem].x;
        let prev_pos = frames[self.arm_idx].0[last_elem - 1].x;
        let x_val: f64 = ee_pos - prev_pos;
        self.loss_fn.compute(x_val)
    }

    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        // Implement a lightweight version if needed
        0.0
    }
}

#[derive(Debug)]
pub struct VerticalArm<F: LossFunction> {
    pub arm_idx: usize,
    pub loss_fn: F,
}

impl<F: LossFunction> ObjectiveTrait for VerticalArm<F> {
    #[inline]
    fn call(&self, _x: &[f64], _v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        // let last_elem = frames[self.arm_idx].0.len() - 1;
        // let euler = frames[0].1[last_elem].euler_angles();
        // println!("{} {}",euler.0,euler.2);
        // (self.loss_fn)(euler.0) + (self.loss_fn)(euler.2-1.57075)

        let last_elem = frames[self.arm_idx].0.len() - 1;
        let y_delta: f64 =
            frames[self.arm_idx].0[last_elem].y - frames[self.arm_idx].0[last_elem - 1].y;

        // et ee_pos = frames[self.arm_idx].0[last_elem];
        // let prev_pos = frames[self.arm_idx].0[last_elem - 1];
        // let x_val: f64 = (ee_pos.x - prev_pos.x).abs() + (ee_pos.y - prev_pos.y).abs();
        self.loss_fn.compute(y_delta)
    }
    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        // let ee_pos = ee_poses[self.arm_idx].0;
        // let   goal = v.goal_positions[self.arm_idx];
        let x_val = 1.0; // placeholder
        self.loss_fn.compute(x_val)
    }
}

#[derive(Debug)]
pub struct VerticalArm2<F: LossFunction> {
    pub arm_idx: usize,
    pub loss_fn: F,
}

impl<F: LossFunction> ObjectiveTrait for VerticalArm2<F> {
    #[inline]
    fn call(&self, _x: &[f64], _v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        // let last_elem = frames[self.arm_idx].0.len() - 1;
        // let euler = frames[0].1[last_elem].euler_angles();
        // println!("{} {}",euler.0,euler.2);
        // (self.loss_fn)(euler.0) + (self.loss_fn)(euler.2-1.57075)

        let last_elem = frames[self.arm_idx].0.len() - 1;
        let x_delta: f64 =
            frames[self.arm_idx].0[last_elem].x - frames[self.arm_idx].0[last_elem - 1].x;
        // et ee_pos = frames[self.arm_idx].0[last_elem];
        // let prev_pos = frames[self.arm_idx].0[last_elem - 1];
        // let x_val: f64 = (ee_pos.x - prev_pos.x).abs() + (ee_pos.y - prev_pos.y).abs();
        self.loss_fn.compute(x_delta)
    }
    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        // let ee_pos = ee_poses[self.arm_idx].0;
        // let   goal = v.goal_positions[self.arm_idx];
        let x_val = 1.0; // placeholder
        self.loss_fn.compute(x_val)
    }
}

#[derive(Debug)]
pub struct HorizontalArm<F: LossFunction> {
    pub arm_idx: usize,
    pub loss_fn: F,
}

impl<F: LossFunction> ObjectiveTrait for HorizontalArm<F> {
    #[inline]
    fn call(&self, _x: &[f64], _v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let ee_pos = frames[self.arm_idx].0[last_elem].z;
        let prev_pos = frames[self.arm_idx].0[last_elem - 1].z;
        let x_val: f64 = ee_pos - prev_pos;
        // et ee_pos = frames[self.arm_idx].0[last_elem];
        // let prev_pos = frames[self.arm_idx].0[last_elem - 1];
        // let x_val: f64 = (ee_pos.x - prev_pos.x).abs() + (ee_pos.y - prev_pos.y).abs();
        self.loss_fn.compute(x_val)
    }
    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        // let ee_pos = ee_poses[self.arm_idx].0;
        // let   goal = v.goal_positions[self.arm_idx];
        let x_val = 1.0; // placeholder
        self.loss_fn.compute(x_val)
    }
}

pub struct HorizontalGripper<F: LossFunction> {
    pub arm_idx: usize,
    pub loss_fn: F,
}

impl<F: LossFunction> ObjectiveTrait for HorizontalGripper<F> {
    #[inline]
    fn call(&self, _x: &[f64], _v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let euler = frames[0].1[last_elem].euler_angles();
        self.loss_fn.compute(euler.1)
    }

    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, ee_poses: &[SinglePose]) -> f64 {
        let euler = ee_poses[self.arm_idx].1.euler_angles();
        self.loss_fn.compute(euler.1)
    }
}

pub struct MatchEEPosiDoF<F: LossFunction> {
    pub arm_idx: usize,
    pub axis: usize,
    pub loss_fn: F,
}
impl<F: LossFunction> ObjectiveTrait for MatchEEPosiDoF<F> {
    #[inline]
    fn call(&self, _x: &[f64], v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let goal_quat = v.goal_quats[self.arm_idx];
        // E_{gc} = R_{gw} * T_{gw} * T_{wc} * R_{wc}, R_{wc} won't matter since we are only interested in the translation
        // so  we get: T_{gc} = R_{gw} * T_{gw} * T_{wc}
        let t_gw_t_wc = Vector3::new(
            frames[self.arm_idx].0[last_elem].x - v.goal_positions[self.arm_idx].x,
            frames[self.arm_idx].0[last_elem].y - v.goal_positions[self.arm_idx].y,
            frames[self.arm_idx].0[last_elem].z - v.goal_positions[self.arm_idx].z,
        );

        let t_gc = goal_quat.inverse() * t_gw_t_wc;
        let dist: f64 = t_gc[self.axis];
        // let bound = v.tolerances[self.arm_idx][self.axis];
        self.loss_fn.compute(dist)
    }
    fn call_lite(&self, _x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &[SinglePose]) -> f64 {
        let x_val = (ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx]).norm();
        self.loss_fn.compute(x_val)
    }
}

pub struct SelfCollision<F: LossFunction> {
    pub arm_idx: usize,
    pub first_link: usize,
    pub second_link: usize,
    pub loss_fn: F,
}

impl<F: LossFunction> ObjectiveTrait for SelfCollision<F> {
    #[inline]
    fn call(&self, x: &[f64], _v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        for x_i in x {
            if x_i.is_nan() {
                return 10.0;
            }
        }

        // let mut x_val: f64 = 0.0;
        // let link_radius = 0.05;

        let start_pt_1 = nalgebra::Point3::from(frames[self.arm_idx].0[self.first_link]);
        let end_pt_1 = nalgebra::Point3::from(frames[self.arm_idx].0[self.first_link + 1]);
        let segment_1 = shape::Segment::new(start_pt_1, end_pt_1);

        let start_pt_2 = nalgebra::Point3::from(frames[self.arm_idx].0[self.second_link]);
        let end_pt_2 = nalgebra::Point3::from(frames[self.arm_idx].0[self.second_link + 1]);
        let segment_2 = shape::Segment::new(start_pt_2, end_pt_2);

        let segment_pos = nalgebra::one();
        // println!("start_pt_1:{} end_pt_1:{}  start_pt_2:{} end_pt_2:{} x: {:?}", start_pt_1, end_pt_1, start_pt_2, end_pt_2, x);

        let dis =
            query::distance(&segment_pos, &segment_1, &segment_pos, &segment_2).unwrap() - 0.05;
        self.loss_fn.compute(dis)
    }

    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        self.loss_fn.compute(1.0)
    }
}

pub struct MaximizeManipulability<F: LossFunction> {
    pub loss_fn: F,
}
impl<F: LossFunction> ObjectiveTrait for MaximizeManipulability<F> {
    #[inline]
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &[Pose]) -> f64 {
        let x_val = v.robot.get_manipulability_with_frame(x, frames);

        self.loss_fn.compute(x_val)
    }

    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        0.0
    }
}
pub struct EachJointLimits<F: LossFunction> {
    pub joint_idx: usize,
    pub loss_fn: F,
}
impl<F: LossFunction> ObjectiveTrait for EachJointLimits<F> {
    #[inline]
    fn call(&self, x: &[f64], _v: &vars::RelaxedIKVars, _frames: &[Pose]) -> f64 {
        self.loss_fn.compute(x[self.joint_idx])
    }

    fn call_lite(&self, _x: &[f64], _v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        0.0
    }
}

pub struct MinimizeVelocity<F: LossFunction> {
    pub loss_fn: F,
}
impl<F: LossFunction> ObjectiveTrait for MinimizeVelocity<F> {
    #[inline]
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, _frames: &[Pose]) -> f64 {
        let x_val = x
            .iter()
            .zip(v.xopt.iter())
            .map(|(x_i, xopt_i)| (x_i - xopt_i).powi(2))
            .sum::<f64>()
            .sqrt();
        self.loss_fn.compute(x_val)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        let x_val = x
            .iter()
            .zip(v.xopt.iter())
            .map(|(x_i, xopt_i)| (x_i - xopt_i).powi(2))
            .sum::<f64>()
            .sqrt();
        self.loss_fn.compute(x_val)
    }
}

pub struct MinimizeAcceleration<F: LossFunction> {
    pub loss_fn: F,
}
impl<F: LossFunction> ObjectiveTrait for MinimizeAcceleration<F> {
    #[inline]
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, _frames: &[Pose]) -> f64 {
        let x_val = x
            .iter()
            .zip(v.prev_state.iter())
            .map(|(x_i, prev_state_i)| (x_i - prev_state_i).powi(2))
            .sum::<f64>()
            .sqrt();
        self.loss_fn.compute(x_val)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        let x_val = x
            .iter()
            .zip(v.xopt.iter())
            .map(|(xi, xopt_i)| (xi - xopt_i).powi(2))
            .sum::<f64>()
            .sqrt();
        self.loss_fn.compute(x_val)
    }
}

pub struct MinimizeJerk<F: LossFunction> {
    pub loss_fn: F,
}
impl<F: LossFunction> ObjectiveTrait for MinimizeJerk<F> {
    #[inline]
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, _frames: &[Pose]) -> f64 {
        /* logic:
            let v1 = xi - xopt_i;
            let v2 = xopt_i - prev_state_i;
            let v3 = prev_state_i - prev_state2_i;
            let a1 = v1 - v2; // xi - xopt_i - (xopt_i - prev_state_i)
                -> xi + prev_state_i -2 *xopt_i
            let a2 = v2 - v3; // xopt_i - prev_state_i - (prev_state_i - prev_state2_i)
                -> xopt_i + prev_state2_i - 2*prev_state_i
            (a1 - a2).powi(2) // xi + prev_state_i -2 *xopt_i - (xopt_i + prev_state2_i - 2*prev_state_i)
                ->xi + 3*prev_state_i -3*xopt_i - prev_state2_i
        */
        let x_val = x
            .iter()
            .zip(v.xopt.iter())
            .zip(v.prev_state.iter())
            .zip(v.prev_state2.iter())
            .map(|(((xi, xopt_i), prev_state_i), prev_state2_i)| {
                (xi + 3.0 * prev_state_i - 3.0 * xopt_i - prev_state2_i).powi(2)
            })
            .sum::<f64>()
            .sqrt();
        self.loss_fn.compute(x_val)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, _ee_poses: &[SinglePose]) -> f64 {
        let x_val = x
            .iter()
            .zip(v.xopt.iter())
            .zip(v.prev_state.iter())
            .zip(v.prev_state2.iter())
            .map(|(((xi, xopt_i), prev_state_i), prev_state2_i)| {
                (xi + 3.0 * prev_state_i - 3.0 * xopt_i - prev_state2_i).powi(2)
            })
            .sum::<f64>()
            .sqrt();
        self.loss_fn.compute(x_val)
    }
}
