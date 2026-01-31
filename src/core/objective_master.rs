use crate::{
    core::{
        loss::{FuncType, SwampType},
        objective::*,
        vars::RelaxedIKVars,
    },
    spacetime::arm::JointLimits,
    utils::structs::*,
};
use {serde::Deserialize, std::fmt::Debug};

/// User configurable part of an objective (loss function & weight)
#[derive(Deserialize, Debug, Clone, Copy)]
pub struct ObjectiveType {
    func: FuncType,
    weight: f64,
}

/// swamp only objective
#[derive(Deserialize, Debug, Clone, Copy)]
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
    pub vertical_arm: Option<ObjectiveType>,
    pub cardinal_directions: Option<ObjectiveType>,
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
    pub vertical_arm: ObjectiveType,
    pub cardinal_directions: ObjectiveType,
    pub joint_limits: ObjectiveSwamp,
    pub minimize_velocity: ObjectiveType,
    pub minimize_acceleration: ObjectiveType,
    pub minimize_jerk: ObjectiveType,
    pub maximize_manipulability: ObjectiveType,
    pub self_collision: ObjectiveType,
}

pub struct ObjectiveMaster {
    pub objectives: Vec<Box<dyn ObjectiveWrapperTrait + Send>>,
    pub num_chains: usize,
    pub lite: bool,
    pub finite_diff_grad: bool,
}

fn add_objective<T>(
    objectives: &mut Vec<Box<dyn ObjectiveWrapperTrait + Send>>,
    objective_config: ObjectiveType,
    objective: T,
) where
    T: ObjectiveTrait + Debug + Send + 'static,
{
    if objective_config.weight > 0.0 {
        objectives.push(Box::new(ObjectiveWrapper {
            objective,
            loss_function: objective_config.func,
            weight: objective_config.weight,
        }));
    }
}

impl ObjectiveMaster {
    pub fn relaxed_ik(
        chain_lengths: &[usize],
        limits: &[JointLimits],
        config: ObjectivesConfig,
    ) -> Self {
        let mut objectives: Vec<Box<dyn ObjectiveWrapperTrait + Send>> = Vec::new();
        let num_chains = chain_lengths.len();

        macro_rules! add_obj {
            ($obj:expr, $obj_struct:expr) => {{
                add_objective(&mut objectives, $obj, $obj_struct);
            }};
        }

        for arm_idx in 0..chain_lengths.len() {
            // axis Z=0; Y=1; X=2;
            add_obj!(config.z_pos, MatchEEPosiDoF { arm_idx, axis: 0 });
            add_obj!(config.y_pos, MatchEEPosiDoF { arm_idx, axis: 1 });
            add_obj!(config.x_pos, MatchEEPosiDoF { arm_idx, axis: 2 });
            add_obj!(config.horizontal_arm, HorizontalArm { arm_idx });
            add_obj!(config.horizontal_grip, HorizontalGripper { arm_idx });
            add_obj!(config.vertical_arm, VerticalArm { arm_idx });
            add_obj!(config.vertical_arm, VerticalArm2 { arm_idx });
            add_obj!(
                config.cardinal_directions,
                CardinalDirectionObjective { arm_idx }
            );
        }
        let SwampType::Swamp(mut params) = config.joint_limits.func;
        for (joint_idx, limit) in limits.iter().enumerate() {
            if limit.lower_bound < -999.0 && limit.upper_bound > 999.0 {
                continue; // ignore joint limit
            }
            params.l_bound = limit.lower_bound;
            params.u_bound = limit.upper_bound;

            add_obj!(
                ObjectiveType {
                    func: FuncType::Swamp(params),
                    weight: config.joint_limits.weight,
                },
                EachJointLimits { joint_idx }
            );
        }

        add_obj!(config.minimize_velocity, MinimizeVelocity);
        add_obj!(config.minimize_acceleration, MinimizeAcceleration);
        add_obj!(config.minimize_jerk, MinimizeJerk);
        add_obj!(config.maximize_manipulability, MaximizeManipulability);

        for (arm_idx, &chain_length) in chain_lengths.iter().enumerate() {
            if chain_length >= 2 {
                for first_link in 0..chain_length - 2 {
                    for second_link in first_link + 2..chain_length {
                        add_obj!(
                            config.self_collision,
                            SelfCollision {
                                arm_idx,
                                first_link,
                                second_link
                            }
                        );
                    }
                }
            }
        }

        log::info!(
            "Loaded objective: \n{}",
            objectives
                .iter()
                .map(|obj| obj.recap())
                .collect::<Vec<_>>()
                .join("\n")
        );
        Self {
            objectives,
            num_chains,
            lite: false,
            finite_diff_grad: true,
        }
    }

    pub fn get_costs(&self, x: &[f64], vars: &RelaxedIKVars) -> Vec<f64> {
        let frames = vars.robot.get_frames_immutable(x);
        let mut out = vec![0.0_f64; self.objectives.len()];
        for (i, objective) in self.objectives.iter().enumerate() {
            out[i] = objective.call(x, vars, &frames);
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

    // pub fn gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
    //     if self.lite {
    //         if self.finite_diff_grad {
    //             self.__gradient_finite_diff_lite(x, vars)
    //         } else {
    //             self.__gradient_lite(x, vars)
    //         }
    //     } else if self.finite_diff_grad {
    //         self.optimized_grad(x, vars)
    //         // self.__gradient_finite_diff(x, vars)
    //     } else {
    //         self.__gradient(x, vars)
    //     }
    // }

    // pub fn gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
    //     if self.lite {
    //         self.__gradient_finite_diff_lite(x, vars)
    //     } else {
    //         self.__gradient_finite_diff(x, vars)
    //     }
    // }

    fn __call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let frames = vars.robot.get_frames_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.objectives[i].call(x, vars, &frames);
        }
        out
    }

    fn __call_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let poses = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.objectives[i].call_lite(x, vars, &poses);
        }
        out
    }

    // fn __gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
    //     let mut grad: Vec<f64> = vec![0.; x.len()];
    //     let mut obj = 0.0;

    //     let mut finite_diff_list: Vec<usize> = Vec::new();
    //     let mut f_0s: Vec<f64> = Vec::new();
    //     let frames_0 = vars.robot.get_frames_immutable(x);
    //     for i in 0..self.objectives.len() {
    //         if self.objectives[i].gradient_type() == 0 {
    //             let (local_obj, local_grad) = self.objectives[i].gradient(x, vars, &frames_0);
    //             f_0s.push(local_obj);
    //             obj += local_obj;
    //             for j in 0..local_grad.len() {
    //                 grad[j] += local_grad[j];
    //             }
    //         } else if self.objectives[i].gradient_type() == 1 {
    //             finite_diff_list.push(i);
    //             let local_obj = self.objectives[i].call(x, vars, &frames_0);
    //             obj += local_obj;
    //             f_0s.push(local_obj);
    //         }
    //     }

    //     println!("before");
    //     if !finite_diff_list.is_empty() {
    //         println!("using finite");
    //         for i in 0..x.len() {
    //             let mut x_h = x.to_vec();
    //             x_h[i] += 0.0000001;
    //             let frames_h = vars.robot.get_frames_immutable(x_h.as_slice());
    //             for &j in &finite_diff_list {
    //                 let f_h = self.objectives[j].call(&x_h, vars, &frames_h);
    //                 grad[i] += ((-f_0s[j] + f_h) / 0.0000001);
    //             }
    //         }
    //     }

    //     (obj, grad)
    // }

    // fn __gradient_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
    //     let mut grad: Vec<f64> = vec![0.; x.len()];
    //     let mut obj = 0.0;

    //     let mut finite_diff_list: Vec<usize> = Vec::new();
    //     let mut f_0s: Vec<f64> = Vec::new();
    //     let poses_0 = vars.robot.get_ee_pos_and_quat_immutable(x);
    //     for i in 0..self.objectives.len() {
    //         if self.objectives[i].gradient_type() == 1 {
    //             let (local_obj, local_grad) = self.objectives[i].gradient_lite(x, vars, &poses_0);
    //             f_0s.push(local_obj);
    //             obj += local_obj;
    //             for j in 0..local_grad.len() {
    //                 grad[j] += local_grad[j];
    //             }
    //         } else if self.objectives[i].gradient_type() == 0 {
    //             finite_diff_list.push(i);
    //             let local_obj = self.objectives[i].call_lite(x, vars, &poses_0);
    //             obj += local_obj;
    //             f_0s.push(local_obj);
    //         }
    //     }

    //     if !finite_diff_list.is_empty() {
    //         for i in 0..x.len() {
    //             let mut x_h = x.to_vec();
    //             x_h[i] += 0.0000001;
    //             let poses_h = vars.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
    //             for j in &finite_diff_list {
    //                 let f_h = self.objectives[*j].call_lite(x, vars, &poses_h);
    //                 grad[i] += ((-f_0s[*j] + f_h) / 0.0000001);
    //             }
    //         }
    //     }

    //     (obj, grad)
    // }

    // fn __gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
    //     let mut grad: Vec<f64> = vec![0.; x.len()];
    //     let f_0 = self.call(x, vars);

    //     for i in 0..x.len() {
    //         let mut x_h = x.to_vec();
    //         x_h[i] += 0.000001;
    //         let f_h = self.call(x_h.as_slice(), vars);
    //         grad[i] = (-f_0 + f_h) / 0.000001;
    //     }

    //     (f_0, grad)
    // }

    /// Calculating only partial frames to improve efficiency
    pub fn optimized_grad(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        // TODO implement multi arm
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let (frame_pos, frame_rot) = vars.robot.arms[0].get_frames_immutable(x);
        let frame_org: Vec<Pose> = vec![(frame_pos.clone(), frame_rot.clone())];
        let mut f_0 = 0.0;
        for i in 0..self.objectives.len() {
            f_0 += self.objectives[i].call(x, vars, &frame_org);
        }

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let guard_frame: Pose = vars.robot.arms[0].get_partial_frames_immutable(
                &x_h,
                frame_pos.clone(),
                frame_rot.clone(),
                i,
            );
            let frame_org = vec![guard_frame];
            let mut f_h = 0.0;
            for j in 0..self.objectives.len() {
                f_h += self.objectives[j].call(&x_h, vars, &frame_org);
            }
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
