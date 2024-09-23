use crate::groove::loss::{self, FuncType, SwampType};
use crate::groove::objective::*;
use crate::groove::vars::RelaxedIKVars;
use serde::Deserialize;
use std::fmt::Debug;


/// User configurable part of an objective (loss function & weight)
#[derive(Deserialize, Debug, Clone)]
pub struct ObjectiveType {
    func: FuncType,
    weight: f64,
}

#[derive(Deserialize, Debug, Clone)]
pub struct ObjectiveSwamp {
    func: SwampType,
    weight: f64,
}

/// Same as ObjectivesConfig but with options to allow for partial input in config
#[derive(Deserialize, Debug, Clone)]
pub struct ObjectivesConfigParse {
    pub x_pos: Option<ObjectiveType>,
    pub y_pos: Option<ObjectiveType>,
    pub z_pos: Option<ObjectiveType>,
    pub horizontal_grip: Option<ObjectiveType>,
    pub horizontal_arm: Option<ObjectiveType>,
    pub joint_limits: Option<ObjectiveSwamp>,
    pub minimize_velocity: Option<ObjectiveType>,
    pub minimize_acceleration: Option<ObjectiveType>,
    pub minimize_jerk: Option<ObjectiveType>,
    pub maximize_manipulability: Option<ObjectiveType>,
    pub self_collision: Option<ObjectiveType>,
}

/// All possible objectives
#[derive(Deserialize, Debug, Clone)]
pub struct ObjectivesConfig {
    pub x_pos: ObjectiveType,
    pub y_pos: ObjectiveType,
    pub z_pos: ObjectiveType,
    pub horizontal_grip: ObjectiveType,
    pub horizontal_arm: ObjectiveType,
    pub joint_limits: ObjectiveSwamp,
    pub minimize_velocity: ObjectiveType,
    pub minimize_acceleration: ObjectiveType,
    pub minimize_jerk: ObjectiveType,
    pub maximize_manipulability: ObjectiveType,
    pub self_collision: ObjectiveType,
}

/// creates a boxed objective
/// * First parameter : FuncType
/// * Second parameter : ObjectiveTrait struct
/// * All other parameter : needed variables to create the struct
///     * Warning: needs to have the same name as the struct fields
/// ## Example :
/// ```
/// let obj:ObjectiveType = ...
/// box_in!(obj.func, MatchEEPosiDoF, arm_idx, axis)
/// // or
/// box_in!(obj.func, MinimizeVelocity)
/// ```
macro_rules! box_in {
    ($func:expr, $obj_struct:ident) => {{
        let bx : Box<dyn ObjectiveTrait + Send> = match $func {
            FuncType::Swamp(p)       => {let loss_fn = move |x| loss::swamp_loss(x, p);        Box::new($obj_struct{loss_fn})},
            FuncType::SwampGroove(p) => {let loss_fn = move |x| loss::swamp_groove_loss(x, p); Box::new($obj_struct{loss_fn})},
            FuncType::Groove(p)      => {let loss_fn = move |x| loss::groove_loss(x, p);       Box::new($obj_struct{loss_fn})},
        };
        bx
    }};
    ($func:expr, $obj_struct:ident $(, $params:tt)*) => {{
        let bx : Box<dyn ObjectiveTrait + Send> = match $func {
            FuncType::Swamp(p)       => {let loss_fn = move |x| loss::swamp_loss(x, p);        Box::new($obj_struct{$( $params ),*, loss_fn})},
            FuncType::SwampGroove(p) => {let loss_fn = move |x| loss::swamp_groove_loss(x, p); Box::new($obj_struct{$( $params ),*, loss_fn})},
            FuncType::Groove(p)      => {let loss_fn = move |x| loss::groove_loss(x, p);       Box::new($obj_struct{$( $params ),*, loss_fn})},
        };
        bx
    }};
}

pub struct ObjectiveMaster {
    pub objectives: Vec<Box<dyn ObjectiveTrait + Send>>,
    pub num_chains: usize,
    pub weight_priors: Vec<f64>,
    pub lite: bool,
    pub finite_diff_grad: bool,
}

impl ObjectiveMaster {
    pub fn relaxed_ik(
        chain_lengths: &[usize],
        lower_joint_limits: &[f64],
        upper_joint_limits: &[f64],
        config: ObjectivesConfig,
    ) -> Self {
        let mut objectives: Vec<Box<dyn ObjectiveTrait + Send>> = Vec::new();
        let mut weight_priors: Vec<f64> = Vec::new();
        let mut recap: String = String::new();
        let num_chains = chain_lengths.len();

        /// helper macro to add an objective to objectives vec and weight
        macro_rules! add_obj {
            ($obj:expr, $obj_struct:ident) => {{
                if $obj.weight > 0.0 {
                    objectives.push(box_in!($obj.func, $obj_struct));
                    weight_priors.push($obj.weight);
                    recap += format!("{:20} - {:?} - {:?}\n", stringify!($obj_struct), $obj.weight, $obj.func).as_str();
                }
            }};
            ($obj:expr, $obj_struct:ident $(, $params:tt)*) => {{
                if $obj.weight > 0.0 {
                    objectives.push(box_in!($obj.func, $obj_struct, $( $params ),*));
                    weight_priors.push($obj.weight);
                    recap += format!("{:20} - {:?} - {:?}\n", stringify!($obj_struct), $obj.weight, $obj.func).as_str();
                }
            }};

        }

        let num_dof: usize = chain_lengths.iter().sum();
        for arm_idx in 0..num_chains {
            // TODO GO CONST FOR MatchEEPosiDoF
            let axis = 0; // Z=0; Y=1; X=2;
            let obj = config.z_pos.clone();
            add_obj!(obj, MatchEEPosiDoF, arm_idx, axis);
            let axis = 1;
            let obj = config.y_pos.clone();
            add_obj!(obj, MatchEEPosiDoF, arm_idx, axis);
            let axis = 2;
            let obj = config.x_pos.clone();
            add_obj!(obj, MatchEEPosiDoF, arm_idx, axis);
            let obj = config.horizontal_arm.clone();
            add_obj!(obj, HorizontalArm, arm_idx);
            let obj = config.horizontal_grip.clone();
            add_obj!(obj, HorizontalGripper, arm_idx);
        }
        let SwampType::Swamp(mut params) = config.joint_limits.func;
        let weight = config.joint_limits.weight;
        for joint_idx in 0..num_dof {
            let l_bound = lower_joint_limits[joint_idx];
            let u_bound = upper_joint_limits[joint_idx];
            if l_bound == -999.0 && u_bound == 999.0 {
                continue; // ignore joint limit
            }
            params.l_bound = l_bound;
            params.u_bound = u_bound;
            let obj = ObjectiveType {
                func: FuncType::Swamp(params),
                weight,
            };
            add_obj!(obj, EachJointLimits, joint_idx);
        }

        let obj = config.minimize_velocity;
        add_obj!(obj, MinimizeVelocity);
        let obj = config.minimize_acceleration;
        add_obj!(obj, MinimizeAcceleration);
        let obj = config.minimize_jerk;
        add_obj!(obj, MinimizeJerk);
        let obj = config.maximize_manipulability;
        add_obj!(obj, MaximizeManipulability);

        let obj = config.self_collision;
        for arm_idx in 0..num_chains {
            if chain_lengths[arm_idx] < 2 {
                continue;
            }
            for first_link in 0..chain_lengths[arm_idx] - 2 {
                for second_link in first_link + 2..chain_lengths[arm_idx] {
                    add_obj!(obj, SelfCollision, arm_idx, first_link, second_link);
                }
            }
        }
        log::info!("objectives : \n{recap}");

        Self {
            objectives,
            num_chains,
            weight_priors,
            lite: false,
            finite_diff_grad: true,
        }
    }

    pub fn get_costs(&self, x: &[f64], vars: &RelaxedIKVars) -> Vec<f64> {
        let frames = vars.robot.get_frames_immutable(x);
        let mut out = vec![0.0_f64; self.objectives.len()];
        for i in 0..self.objectives.len() {
            out[i] = self.weight_priors[i] * self.objectives[i].call(x, vars, &frames);
        }
        out
    }

    pub fn call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        if self.lite {
            self.__call_lite(x, vars)
        } else {
            self.__call(x, vars)
        }
    }

    pub fn gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        if self.lite {
            if self.finite_diff_grad {
                self.__gradient_finite_diff_lite(x, vars)
            } else {
                self.__gradient_lite(x, vars)
            }
        } else if self.finite_diff_grad {
            self.__gradient_finite_diff(x, vars)
        } else {
            self.__gradient(x, vars)
        }
    }

    pub fn gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        if self.lite {
            self.__gradient_finite_diff_lite(x, vars)
        } else {
            self.__gradient_finite_diff(x, vars)
        }
    }

    fn __call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let frames = vars.robot.get_frames_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.weight_priors[i] * self.objectives[i].call(x, vars, &frames);
        }
        out
    }

    fn __call_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let poses = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.weight_priors[i] * self.objectives[i].call_lite(x, vars, &poses);
        }
        out
    }

    fn __gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let frames_0 = vars.robot.get_frames_immutable(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 0 {
                let (local_obj, local_grad) = self.objectives[i].gradient(x, vars, &frames_0);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 1 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call(x, vars, &frames_0);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if !finite_diff_list.is_empty() {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let frames_h = vars.robot.get_frames_immutable(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call(&x_h, vars, &frames_h);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) / 0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let poses_0 = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 1 {
                let (local_obj, local_grad) = self.objectives[i].gradient_lite(x, vars, &poses_0);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 0 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call_lite(x, vars, &poses_0);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if !finite_diff_list.is_empty() {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let poses_h = vars.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call_lite(x, vars, &poses_h);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) / 0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(x, vars);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(x_h.as_slice(), vars);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }

    fn __gradient_finite_diff_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(x, vars);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.__call_lite(x_h.as_slice(), vars);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }
}
