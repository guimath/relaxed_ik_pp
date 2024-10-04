use crate::groove::vars;
// use crate::utils::transformations::*;
// use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::Vector3;
use parry3d_f64::{query, shape};


// For the best perf on a static set of objectives should like at Const Generic

// trait alias equivalent 
pub trait LossFunction: Fn(f64) -> f64 {}
impl <F> LossFunction for F where F:Fn(f64) -> f64{}

pub trait ObjectiveTrait {
    fn call(
        &self,
        x: &[f64],               //joint values
        v: &vars::RelaxedIKVars, // general config variables (like target etx)
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>, // all frames poses
    ) -> f64; // returns loss value

    fn call_lite(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64;
    fn gradient(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> (f64, Vec<f64>) {
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
        ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
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

#[derive(Debug)]
pub struct HorizontalArm<F: LossFunction>{
    pub arm_idx: usize,
    pub loss_fn: F,
}

impl <F:LossFunction> ObjectiveTrait for HorizontalArm<F> 

    {
    #[inline]
    fn call(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let ee_pos = frames[self.arm_idx].0[last_elem].z;
        let prev_pos = frames[self.arm_idx].0[last_elem - 1].z;
        let x_val: f64 = ee_pos - prev_pos;
        (self.loss_fn)(x_val)
    }
    fn call_lite(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        // let ee_pos = ee_poses[self.arm_idx].0;
        // let   goal = v.goal_positions[self.arm_idx];
        let x_val = 1.0; // placeholder
        (self.loss_fn)(x_val)
    }
}

pub struct HorizontalGripper<F: LossFunction> {
    pub arm_idx: usize,
    pub loss_fn: F,
}

impl <F:LossFunction> ObjectiveTrait for HorizontalGripper<F> {
    #[inline]
    fn call(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let euler = frames[0].1[last_elem].euler_angles();
        (self.loss_fn)(euler.1)
    }

    fn call_lite(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        let euler = ee_poses[self.arm_idx].1.euler_angles();
        (self.loss_fn)(euler.1)
    }
}

pub struct MatchEEPosiDoF<F: LossFunction>  {
    pub arm_idx: usize,
    pub axis: usize,
    pub loss_fn: F,
}
impl <F: LossFunction>  ObjectiveTrait for MatchEEPosiDoF<F>{
    #[inline]
    fn call(
        &self,
        _x: &[f64],
        v: &vars::RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
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
        (self.loss_fn)(dist)
    }
    fn call_lite(
        &self,
        _x: &[f64],
        v: &vars::RelaxedIKVars,
        ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        let x_val = (ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx]).norm();
        (self.loss_fn)(x_val)
    }
}


pub struct SelfCollision<F: LossFunction> {
    pub arm_idx: usize,
    pub first_link: usize,
    pub second_link: usize,
    pub loss_fn: F
}

impl <F: LossFunction> ObjectiveTrait for SelfCollision<F> {
    #[inline]
    fn call(
        &self,
        x: &[f64],
        _v: &vars::RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        for i in 0..x.len() {
            if x[i].is_nan() {
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
        (self.loss_fn)(dis)
        
    }

    fn call_lite(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        (self.loss_fn)(1.0) // placeholder
    }
}


pub struct MaximizeManipulability<F: LossFunction> {
    pub loss_fn: F
}
impl <F: LossFunction> ObjectiveTrait for MaximizeManipulability<F> {
    #[inline]
    fn call(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        let x_val = v.robot.get_manipulability_with_frame(x, &frames);


        (self.loss_fn)(x_val)
    }

    fn call_lite(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        0.0
    }
}
pub struct EachJointLimits<F: LossFunction> {
    pub joint_idx: usize,
    pub loss_fn: F
}
impl <F:LossFunction> ObjectiveTrait for EachJointLimits<F> {
    #[inline]
    fn call(
        &self,
        x: &[f64],
        _v: &vars::RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        (self.loss_fn)(x[self.joint_idx])
    }

    fn call_lite(
        &self,
        _x: &[f64],
        _v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        0.0
    }
}

pub struct MinimizeVelocity <F: LossFunction> {
    pub loss_fn: F
}
impl <F: LossFunction> ObjectiveTrait for MinimizeVelocity<F> {
    #[inline]
    fn call(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        (self.loss_fn)(x_val)
    }

    fn call_lite(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        
        (self.loss_fn)(x_val)
    }
}

pub struct MinimizeAcceleration<F: LossFunction> {
    pub loss_fn: F
}
impl <F: LossFunction> ObjectiveTrait for MinimizeAcceleration<F> {
    #[inline]
    fn call(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        (self.loss_fn)(x_val)
    }

    fn call_lite(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        (self.loss_fn)(x_val)
    }
}

pub struct MinimizeJerk<F: LossFunction>{
    pub loss_fn: F
}
impl <F: LossFunction> ObjectiveTrait for MinimizeJerk<F> {
    #[inline]
    fn call(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        (self.loss_fn)(x_val)
    }

    fn call_lite(
        &self,
        x: &[f64],
        v: &vars::RelaxedIKVars,
        _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        (self.loss_fn)(x_val)
    }
}



// pub struct TargetCollision {
//     pub arm_idx: usize,
//     pub link: usize,
//     pub loss_fn: Box<dyn Fn(f64) -> f64>
// }

// impl ObjectiveTrait for TargetCollision {
//     #[inline]
//     fn call(
//         &self,
//         x: &[f64],
//         v: &vars::RelaxedIKVars,
//         frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
//     ) -> f64 {
//         for i in 0..x.len() {
//             if x[i].is_nan() {
//                 return 10.0;
//             }
//         }

//         // let mut x_val: f64 = 0.0;
//         // let link_radius = 0.05;

//         let start_pt_1 = nalgebra::Point3::from(frames[self.arm_idx].0[self.link]);
//         let end_pt_1 = nalgebra::Point3::from(frames[self.arm_idx].0[self.link + 1]);
//         let segment_1 = shape::Segment::new(start_pt_1, end_pt_1);

//         let goal_center: Vector3<f64> = v.goal_positions[self.arm_idx];
//         let goal_top = Vector3::new(goal_center.x, goal_center.y, goal_center.z + 1.0);
//         let goal_bot = Vector3::new(goal_center.x, goal_center.y, goal_center.z - 1.0);

//         let start_pt_2 = nalgebra::Point3::from(goal_top);
//         let end_pt_2 = nalgebra::Point3::from(goal_bot);
//         let segment_2 = shape::Segment::new(start_pt_2, end_pt_2);

//         let segment_pos = nalgebra::one();
//         // println!("start_pt_1:{} end_pt_1:{}  start_pt_2:{} end_pt_2:{} x: {:?}", start_pt_1, end_pt_1, start_pt_2, end_pt_2, x);

//         let dis =
//             query::distance(&segment_pos, &segment_1, &segment_pos, &segment_2).unwrap() - 0.05;
//         (self.loss_fn)(dis)

//     }

//     fn call_lite(
//         &self,
//         _x: &[f64],
//         _v: &vars::RelaxedIKVars,
//         _ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
//     ) -> f64 {
//         (self.loss_fn)(1.0)// placeholder
//     }
// }

// pub struct MatchEERotaDoF {
//     pub arm_idx: usize,
//     pub axis: usize,
// }
// impl MatchEERotaDoF {
//     pub fn new(arm_idx: usize, axis: usize) -> Self {
//         Self { arm_idx, axis }
//     }
// }
// impl ObjectiveTrait for MatchEERotaDoF {
//     #[inline]
//     fn call(
//         &self,
//         _x: &[f64],
//         v: &vars::RelaxedIKVars,
//         frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
//     ) -> f64 {
//         let last_elem = frames[self.arm_idx].1.len() - 1;
//         let ee_quat = frames[self.arm_idx].1[last_elem];
//         let goal_quat = v.goal_quats[self.arm_idx];
//         let rotation = goal_quat.inverse() * ee_quat;

//         // let euler = rotation.euler_angles();
//         // println!("axisAngle: {:?} {:?}", euler, axisAngle);
//         let scaled_axis = rotation.scaled_axis();

//         let mut angle: f64 = 0.0;
//         angle += scaled_axis[self.axis].abs();

//         let bound = v.tolerances[self.arm_idx][self.axis + 3];

//         if bound <= 1e-2 {
//             groove_loss(angle, 0., 2, 0.1, 10.0, 2)
//         } else if bound >= 3.14159260 {
//             swamp_loss(angle, -bound, bound, 100.0, 0.1, 20)
//         } else {
//             swamp_groove_loss(angle, 0.0, -bound, bound, bound * 2.0, 1.0, 0.01, 100.0, 20)
//             // swamp_groove_loss(angle, 0.0, -bound, bound, 10.0, 1.0, 0.01, 100.0, 20)
//         }
//     }

//     fn call_lite(
//         &self,
//         _x: &[f64],
//         v: &vars::RelaxedIKVars,
//         ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
//     ) -> f64 {
//         let x_val = (ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx]).norm();
//         groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
//     }
// }

// pub struct EnvCollision {
//     pub arm_idx: usize
// }
// impl EnvCollision {
//     pub fn new(arm_idx: usize) -> Self {Self{arm_idx}}
// }
// impl ObjectiveTrait for EnvCollision {
//     fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
//         // let start = PreciseTime::now();\

//         for i in 0..x.len() {
//             if (x[i].is_nan()) {
//                 return 10.0
//             }
//         }

//         let mut x_val: f64 = 0.0;
//         let link_radius = v.env_collision.link_radius;
//         let penalty_cutoff: f64 = link_radius * 2.0;
//         let a = penalty_cutoff.powi(2);
//         for (option, score) in &v.env_collision.active_obstacles[self.arm_idx] {
//             if let Some(handle) = option {
//                 let mut sum: f64 = 0.0;
//                 let obstacle = v.env_collision.world.objects.get(*handle).unwrap();
//                 let last_elem = frames[self.arm_idx].0.len() - 1;
//                 for i in 0..last_elem {
//                     let mut start_pt = Point3::from(frames[self.arm_idx].0[i]);
//                      // hard coded for ur5
//                     if (i == last_elem - 1) {
//                         start_pt = Point3::from(frames[self.arm_idx].0[i] + 0.2 * (frames[self.arm_idx].0[i] - frames[self.arm_idx].0[i + 1]));
//                     }

//                     let end_pt = Point3::from(frames[self.arm_idx].0[i + 1]);
//                     let segment = shape::Segment::new(start_pt, end_pt);
//                     let segment_pos = nalgebra::one();
//                     let dis = query::distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius;
//                     // println!("Obstacle: {}, Link: {}, Distance: {:?}", obstacle.data().name, i, dis);
//                     sum += a / (dis + link_radius).powi(2);
//                 }
//                 // println!("OBJECTIVE -> {:?}, Sum: {:?}", obstacle.data().name, sum);
//                 x_val += sum;
//             }
//         }

//         // let end = PreciseTime::now();
//         // println!("Obstacles calculating takes {}", start.to(end));

//         groove_loss(x_val, 0., 2, 3.5, 0.00005, 4)
//     }

//     fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
//         let x_val = 1.0; // placeholder
//         groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
//     }
// }

// pub struct MatchEEPosGoals {
//     pub arm_idx: usize,
// }
// impl ObjectiveTrait for MatchEEPosGoals {
//     #[inline]
//     fn call(
//         &self,
//         _x: &[f64],
//         v: &vars::RelaxedIKVars,
//         frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
//     ) -> f64 {
//         let last_elem = frames[self.arm_idx].0.len() - 1;
//         let x_val = (frames[self.arm_idx].0[last_elem] - v.goal_positions[self.arm_idx]).norm();

//         groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
//     }

//     fn call_lite(
//         &self,
//         _x: &[f64],
//         v: &vars::RelaxedIKVars,
//         ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
//     ) -> f64 {
//         let x_val = (ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx]).norm();
//         groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
//     }
// }

// pub struct MatchEEQuatGoals {
//     pub arm_idx: usize,
// }
// impl MatchEEQuatGoals {
//     pub fn new(arm_idx: usize) -> Self {
//         Self { arm_idx }
//     }
// }
// impl ObjectiveTrait for MatchEEQuatGoals {
//     #[inline]
//     fn call(
//         &self,
//         _x: &[f64],
//         v: &vars::RelaxedIKVars,
//         frames: &Vec<(Vec<Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
//     ) -> f64 {
//         let last_elem = frames[self.arm_idx].1.len() - 1;
//         let tmp = Quaternion::new(
//             -frames[self.arm_idx].1[last_elem].w,
//             -frames[self.arm_idx].1[last_elem].i,
//             -frames[self.arm_idx].1[last_elem].j,
//             -frames[self.arm_idx].1[last_elem].k,
//         );
//         let ee_quat2 = UnitQuaternion::from_quaternion(tmp);

//         let disp = angle_between_quaternion(
//             v.goal_quats[self.arm_idx],
//             frames[self.arm_idx].1[last_elem],
//         );
//         let disp2 = angle_between_quaternion(v.goal_quats[self.arm_idx], ee_quat2);
//         let x_val = disp.min(disp2);

//         groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
//     }

//     fn call_lite(
//         &self,
//         _x: &[f64],
//         v: &vars::RelaxedIKVars,
//         ee_poses: &Vec<(Vector3<f64>, nalgebra::UnitQuaternion<f64>)>,
//     ) -> f64 {
//         let tmp = Quaternion::new(
//             -ee_poses[self.arm_idx].1.w,
//             -ee_poses[self.arm_idx].1.i,
//             -ee_poses[self.arm_idx].1.j,
//             -ee_poses[self.arm_idx].1.k,
//         );
//         let ee_quat2 = UnitQuaternion::from_quaternion(tmp);

//         let disp = angle_between_quaternion(v.goal_quats[self.arm_idx], ee_poses[self.arm_idx].1);
//         let disp2 = angle_between_quaternion(v.goal_quats[self.arm_idx], ee_quat2);
//         let x_val = disp.min(disp2);
//         groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
//     }
// }
