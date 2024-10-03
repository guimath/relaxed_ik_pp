use nalgebra;
use nalgebra::{DMatrix, Unit, UnitQuaternion, Vector3, Vector6};

#[derive(Clone, Debug)]
pub struct RevoluteArm {
    pub num_dof: usize,
    // fixed linear displacements between joints
    pub lin_offsets: Vec<Vector3<f64>>,
    // fixed rotational displacements between joints
    pub rot_offsets: Vec<UnitQuaternion<f64>>,
    is_rot_offset_null: Vec<bool>,
    // Axis of joint
    joint_axis: Vec<Unit<Vector3<f64>>>,

    pub upper_joint_limits: Vec<f64>,
    pub lower_joint_limits: Vec<f64>,
    get_quat: Vec<fn(f64) -> UnitQuaternion<f64>>,
}
impl RevoluteArm {
    pub fn from_chain(
        chain: k::SerialChain<f64>,
    ) -> RevoluteArm {
        let mut num_dof = 0;
        let mut get_quat: Vec<fn(f64) -> UnitQuaternion<f64>> = Vec::new();
        let mut lin_offsets: Vec<Vector3<f64>> = Vec::new();
        let mut rot_offsets: Vec<UnitQuaternion<f64>> = Vec::new();
        let mut upper_joint_limits: Vec<f64> = Vec::new();
        let mut lower_joint_limits: Vec<f64> = Vec::new();
        let mut joint_axis: Vec<Unit<Vector3<f64>>> = Vec::new();
        
        let mut first_link = true;
        let mut add_to_next = false;
        let mut rot_add: UnitQuaternion<f64> = UnitQuaternion::identity();
        let mut lin_add: Vector3<f64> = Vector3::zeros();
        chain.iter().for_each(|node| {
            let joint = node.joint();
            if first_link {
                first_link = false;
                return;
            } 
            match joint.joint_type {
                k::JointType::Fixed => {
                    let org = joint.origin();
                    if add_to_next {
                        rot_add *= org.rotation;
                        lin_add += org.translation.vector;
                    }
                    else {
                        rot_add = org.rotation;
                        lin_add = org.translation.vector;
                    }
                    add_to_next = true;
                }
                k::JointType::Rotational { axis } => {
                    num_dof +=1;
                    joint_axis.push(axis);
                    if *axis == *Vector3::x_axis() {
                        get_quat.push(|val:f64| UnitQuaternion::from_euler_angles(val, 0., 0.))
                    } else if *axis == *Vector3::y_axis() {
                        get_quat.push(|val:f64| UnitQuaternion::from_euler_angles(0., val, 0.))
                    } else if *axis == *Vector3::z_axis() {
                        get_quat.push(|val:f64| UnitQuaternion::from_euler_angles(0., 0., val))
                    } else if *axis == -*Vector3::x_axis() {
                        get_quat.push(|val:f64| UnitQuaternion::from_euler_angles(-val, 0., 0.))
                    } else if *axis == -*Vector3::y_axis() {
                        get_quat.push(|val:f64| UnitQuaternion::from_euler_angles(0., -val, 0.))
                    } else if *axis == -*Vector3::z_axis() {
                        get_quat.push(|val:f64| UnitQuaternion::from_euler_angles(0., 0., -val))
                    } else {
                        panic!("Axis not recognized")
                    }
                   
                    if joint.limits.is_none() {
                        lower_joint_limits.push(-999.0);
                        upper_joint_limits.push(999.0);
                    } else {
                        lower_joint_limits.push(joint.limits.unwrap().min);
                        upper_joint_limits.push(joint.limits.unwrap().max);
                    }
                    let org = joint.origin();
                    lin_offsets.push(org.translation.vector);
                    rot_offsets.push(org.rotation);
                    if add_to_next {
                        lin_offsets[num_dof-1] = lin_add + lin_offsets[num_dof-1];
                        rot_offsets[num_dof-1] = rot_add * rot_offsets[num_dof-1];
                    }
                    add_to_next = false;
                }
                k::JointType::Linear { axis: _ } => {
                    panic!("Linear/prismatic joints not supported in simplified version");
                }
            }
        });

        // adding EE offsets (even if none)
        if add_to_next {
            lin_offsets.push(lin_add);
            rot_offsets.push(rot_add);
        }
        else {
            lin_offsets.push(Vector3::zeros());
            rot_offsets.push(UnitQuaternion::identity())
        }


        let mut is_rot_offset_null: Vec<bool> = Vec::new();
        for i in 0..num_dof+1 {
            let r = rot_offsets[i];
            is_rot_offset_null.push(r[0] == 0.0 && r[1] == 0.0 && r[2] == 0.0)
        } 
        RevoluteArm{
            num_dof,
            lin_offsets,
            rot_offsets,
            is_rot_offset_null,
            joint_axis,
            upper_joint_limits,
            lower_joint_limits,
            get_quat
        }
    }

    pub fn get_frames_immutable(
        &self,
        x: &[f64],
    ) -> (
        Vec<Vector3<f64>>,
        Vec<UnitQuaternion<f64>>,
    ) {
        let mut out_positions: Vec<Vector3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<UnitQuaternion<f64>> = Vec::new();

        let mut pt: Vector3<f64> = Vector3::zeros();
        let mut rot_quat: UnitQuaternion<f64> = UnitQuaternion::identity();

        out_positions.push(pt);
        out_rot_quats.push(rot_quat);

        for i in 0..self.num_dof {
            pt = rot_quat * self.lin_offsets[i] + pt;
            if !self.is_rot_offset_null[i] {
                rot_quat *= self.rot_offsets[i];
            }
            rot_quat *= self.get_quat[i](x[i]);
            out_positions.push(pt);
            out_rot_quats.push(rot_quat);

        }

        //adding EE pose
        out_positions.push(rot_quat*self.lin_offsets[self.num_dof] +pt);
        out_rot_quats.push(rot_quat*self.rot_offsets[self.num_dof]);

        (out_positions, out_rot_quats)
    }
    
    pub fn get_partial_frames_immutable(
        &self,
        x: &[f64],
        mut out_positions: Vec<Vector3<f64>>,
        mut out_rot_quats: Vec<UnitQuaternion<f64>>,
        start:usize

    ) -> (
        Vec<Vector3<f64>>,
        Vec<UnitQuaternion<f64>>,
    ) {
        let mut pt: Vector3<f64> = out_positions[start];
        let mut rot_quat: UnitQuaternion<f64> = out_rot_quats[start];

        for i in start..self.num_dof {
            pt = rot_quat * self.lin_offsets[i] + pt;
            if !self.is_rot_offset_null[i] {
                rot_quat *= self.rot_offsets[i];
            }
            rot_quat *= self.get_quat[i](x[i]);
            out_positions[i+1] = pt;
            out_rot_quats[i+1] = rot_quat;
        }

        //adding EE pose
        out_positions[self.num_dof+1] = rot_quat*self.lin_offsets[self.num_dof] +pt;
        out_rot_quats[self.num_dof+1] = rot_quat*self.rot_offsets[self.num_dof];

        (out_positions, out_rot_quats)
    }
    
    pub fn get_jacobian_immutable(&self, x: &[f64]) -> DMatrix<f64> {
        let (joint_positions, joint_rot_quats) = self.get_frames_immutable(x);
        let ee_position = joint_positions[joint_positions.len() - 1];
        
        let mut jacobian: DMatrix<f64> = DMatrix::identity(6, x.len());
        
        for i in 0..self.num_dof {
            
            let disp = ee_position - joint_positions[i+1];
            let p_axis = joint_rot_quats[i+1] * self.joint_axis[i];
            let linear = p_axis.cross(&disp);
            jacobian.set_column(
                i,
                &Vector6::new(linear.x, linear.y, linear.z, p_axis.x, p_axis.y, p_axis.z),
            );
        }
        jacobian
    }

    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let jacobian = self.get_jacobian_immutable(x);
        (jacobian.clone() * jacobian.transpose())
            .determinant()
            .sqrt()
    }

    pub fn get_ee_pos_and_quat_immutable(
        &self,
        x: &[f64],
    ) -> (Vector3<f64>, UnitQuaternion<f64>) {
        let (joint_positions, joint_rot_quats) = self.get_frames_immutable(x);
        (joint_positions[self.num_dof], joint_rot_quats[self.num_dof])
    }

}

#[derive(Clone, Debug)]
pub struct Arm {
    pub axis_types: Vec<String>,
    pub displacements: Vec<nalgebra::Vector3<f64>>,
    pub rot_offset_quats: Vec<nalgebra::UnitQuaternion<f64>>,
    pub joint_types: Vec<String>,
    pub num_dof: usize,
    pub out_positions: Vec<nalgebra::Vector3<f64>>,
    pub out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>>,
    __do_rot_offset: Vec<bool>,
    __is_prismatic: Vec<bool>,
    __is_revolute_or_continuous: Vec<bool>,
    __is_fixed: Vec<bool>,
    __is_x: Vec<bool>,
    __is_y: Vec<bool>,
    __is_z: Vec<bool>,
    __is_neg_x: Vec<bool>,
    __is_neg_y: Vec<bool>,
    __is_neg_z: Vec<bool>,
    __aux_matrix: nalgebra::Matrix3<f64>,
}

impl Arm {
    pub fn init(
        axis_types: Vec<String>,
        disp_offsets: Vec<nalgebra::Vector3<f64>>,
        rot_offsets: Vec<UnitQuaternion<f64>>,
        joint_types: Vec<String>,
    ) -> Arm {
        let num_dof = axis_types.len();

        let mut __do_rot_offset: Vec<bool> = Vec::new();
        for rot_offset in rot_offsets.clone() {
            if rot_offset[0] == 0.0 && rot_offset[1] == 0.0 && rot_offset[2] == 0.0 {
                __do_rot_offset.push(false);
            } else {
                __do_rot_offset.push(true);
            }
        }

        let displacements: Vec<nalgebra::Vector3<f64>> = disp_offsets.clone();
        // Vec::new();
        // for i in 0..disp_offsets.len() {
        //     displacements.push(disp_offsets[i]);
        // }

        let rot_offset_quats: Vec<nalgebra::UnitQuaternion<f64>> = rot_offsets.clone();

        let mut out_positions: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        for _ in 0..rot_offsets.len() {
            out_positions.push(nalgebra::Vector3::new(0., 0., 0.));
            out_rot_quats.push(nalgebra::UnitQuaternion::identity());
        }

        let mut __is_prismatic: Vec<bool> = Vec::new();
        let mut __is_revolute_or_continuous: Vec<bool> = Vec::new();
        let mut __is_fixed: Vec<bool> = Vec::new();
        for joint_type in joint_types.clone() {
            if joint_type == *"prismatic" {
                __is_prismatic.push(true);
                __is_revolute_or_continuous.push(false);
                __is_fixed.push(false);
            } else if joint_type == *"continuous" || joint_type == *"revolute" {
                __is_prismatic.push(false);
                __is_revolute_or_continuous.push(true);
                __is_fixed.push(false);
            } else if joint_type == *"fixed" {
                __is_prismatic.push(false);
                __is_revolute_or_continuous.push(false);
                __is_fixed.push(true);
            }
        }

        let __aux_matrix: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();

        let mut __is_x: Vec<bool> = Vec::new();
        let mut __is_y: Vec<bool> = Vec::new();
        let mut __is_z: Vec<bool> = Vec::new();
        let mut __is_neg_x: Vec<bool> = Vec::new();
        let mut __is_neg_y: Vec<bool> = Vec::new();
        let mut __is_neg_z: Vec<bool> = Vec::new();
        for i in 0..axis_types.len() {
            __is_x.push(false);
            __is_y.push(false);
            __is_z.push(false);
            __is_neg_x.push(false);
            __is_neg_y.push(false);
            __is_neg_z.push(false);
            if axis_types[i] == *"X" || axis_types[i] == *"x" {
                __is_x[i] = true;
            } else if axis_types[i] == *"Y" || axis_types[i] == *"y" {
                __is_y[i] = true;
            } else if axis_types[i] == *"Z" || axis_types[i] == *"z" {
                __is_z[i] = true;
            } else if axis_types[i] == *"-x" {
                __is_neg_x[i] = true;
            } else if axis_types[i] == *"-y" {
                __is_neg_y[i] = true;
            } else if axis_types[i] == *"-z" {
                __is_neg_z[i] = true;
            }
        }

        // println!("displacements: {:?}", displacements);
        // println!("axis_types: {:?}", axis_types);
        // println!("__is_revolute_or_continuous: {:?}", __is_revolute_or_continuous);
        // println!("__do_rot_offset: {:?}", __do_rot_offset);
        // println!("rot_offset_quats: {:?}", rot_offset_quats);
        // println!("joint_types: {:?}", joint_types);
        Arm {
            axis_types,
            displacements,
            rot_offset_quats,
            joint_types,
            num_dof,
            out_positions,
            out_rot_quats,
            __do_rot_offset,
            __is_prismatic,
            __is_revolute_or_continuous,
            __is_fixed,
            __is_x,
            __is_y,
            __is_z,
            __is_neg_x,
            __is_neg_y,
            __is_neg_z,
            __aux_matrix,
        }
    }

    pub fn get_frames_immutable(
        &self,
        x: &[f64],
    ) -> (
        Vec<nalgebra::Vector3<f64>>,
        Vec<nalgebra::UnitQuaternion<f64>>,
    ) {
        let mut out_positions: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();

        let mut pt: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0., 0., 0.);
        let mut rot_quat = nalgebra::UnitQuaternion::identity();

        out_positions.push(pt);
        out_rot_quats.push(rot_quat);

        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {
            if self.__is_revolute_or_continuous[i] {
                pt = rot_quat * self.displacements[i] + pt;

                if self.__do_rot_offset[i] {
                    rot_quat *= self.rot_offset_quats[i];
                }

                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    rot_quat *= get_quat_x(joint_val);
                } else if self.__is_y[joint_idx] {
                    rot_quat *= get_quat_y(joint_val);
                } else if self.__is_z[joint_idx] {
                    rot_quat *= get_quat_z(joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    rot_quat *= get_quat_x(-joint_val);
                } else if self.__is_neg_y[joint_idx] {
                    rot_quat *= get_quat_y(-joint_val);
                } else if self.__is_neg_z[joint_idx] {
                    rot_quat *= get_quat_z(-joint_val);
                }

                out_positions.push(pt);
                out_rot_quats.push(rot_quat);

                joint_idx += 1;
            } else if self.__is_prismatic[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(joint_val, 0., 0.);
                } else if self.__is_y[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., joint_val, 0.);
                } else if self.__is_z[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., 0., joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(-joint_val, 0., 0.);
                } else if self.__is_neg_y[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., -joint_val, 0.);
                } else if self.__is_neg_z[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., 0., -joint_val);
                }
                if self.__do_rot_offset[i] {
                    rot_quat *= self.rot_offset_quats[i];
                }
                out_positions.push(pt);
                out_rot_quats.push(rot_quat);
                joint_idx += 1;
            } else {
                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i] {
                    rot_quat *= self.rot_offset_quats[i];
                }
                out_positions.push(pt);
                out_rot_quats.push(rot_quat);
            }
        }
        out_rot_quats.push(rot_quat);

        (out_positions, out_rot_quats)
    }

    pub fn get_jacobian_immutable(&self, x: &[f64]) -> DMatrix<f64> {
        let (joint_positions, joint_rot_quats) = self.get_frames_immutable(x);

        let ee_position = joint_positions[joint_positions.len() - 1];
        let pos_x: nalgebra::Vector3<f64> = nalgebra::Vector3::new(1.0, 0.0, 0.0);
        let pos_y: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, 1.0, 0.0);
        let pos_z: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, 0.0, 1.0);
        let neg_x: nalgebra::Vector3<f64> = nalgebra::Vector3::new(-1.0, 0.0, 0.0);
        let neg_y: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, -1.0, 0.0);
        let neg_z: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, 0.0, -1.0);

        // let mut disp: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
        let mut p_axis: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
        let mut joint_idx: usize = 0;

        let mut jacobian: DMatrix<f64> = DMatrix::identity(6, x.len());

        for i in 1..self.displacements.len() {
            if self.__is_revolute_or_continuous[i - 1] {
                let disp = ee_position - joint_positions[i];
                if self.__is_x[joint_idx] {
                    p_axis = joint_rot_quats[i] * pos_x
                } else if self.__is_y[joint_idx] {
                    p_axis = joint_rot_quats[i] * pos_y
                } else if self.__is_z[joint_idx] {
                    p_axis = joint_rot_quats[i] * pos_z
                } else if self.__is_neg_x[joint_idx] {
                    p_axis = joint_rot_quats[i] * neg_x
                } else if self.__is_neg_y[joint_idx] {
                    p_axis = joint_rot_quats[i] * neg_y
                } else if self.__is_neg_z[joint_idx] {
                    p_axis = joint_rot_quats[i] * neg_z
                }

                let linear = p_axis.cross(&disp);
                jacobian.set_column(
                    joint_idx,
                    &Vector6::new(linear.x, linear.y, linear.z, p_axis.x, p_axis.y, p_axis.z),
                );

                joint_idx += 1;
            }
        }

        jacobian
    }

    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let jacobian = self.get_jacobian_immutable(x);
        (jacobian.clone() * jacobian.transpose())
            .determinant()
            .sqrt()
    }

    pub fn get_ee_pos_and_quat_immutable(
        &self,
        x: &[f64],
    ) -> (nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>) {
        let mut pt: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0., 0., 0.);
        let mut rot_quat = nalgebra::UnitQuaternion::identity();

        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {
            if self.__is_revolute_or_continuous[i] {
                pt = rot_quat * self.displacements[i] + pt;

                if self.__do_rot_offset[i] {
                    rot_quat *= self.rot_offset_quats[i];
                }

                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    rot_quat *= get_quat_x(joint_val);
                } else if self.__is_y[joint_idx] {
                    rot_quat *= get_quat_y(joint_val);
                } else if self.__is_z[joint_idx] {
                    rot_quat *= get_quat_z(joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    rot_quat *= get_quat_x(-joint_val);
                } else if self.__is_neg_y[joint_idx] {
                    rot_quat *= get_quat_y(-joint_val);
                } else if self.__is_neg_z[joint_idx] {
                    rot_quat *= get_quat_z(-joint_val);
                }

                joint_idx += 1;
            } else if self.__is_prismatic[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(joint_val, 0., 0.);
                } else if self.__is_y[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., joint_val, 0.);
                } else if self.__is_z[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., 0., joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(-joint_val, 0., 0.);
                } else if self.__is_neg_y[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., -joint_val, 0.);
                } else if self.__is_neg_z[joint_idx] {
                    pt = rot_quat * self.displacements[i]
                        + pt
                        + nalgebra::Vector3::new(0., 0., -joint_val);
                }
                if self.__do_rot_offset[i] {
                    rot_quat *= self.rot_offset_quats[i];
                }
                joint_idx += 1;
            } else {
                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i] {
                    rot_quat *= self.rot_offset_quats[i];
                }
            }
        }

        (pt, rot_quat)
    }
}

pub fn get_rot_x(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(
        1.,
        0.,
        0.,
        0.,
        val.cos(),
        -val.sin(),
        0.0,
        val.sin(),
        val.cos(),
    )
}

pub fn get_rot_y(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(
        val.cos(),
        0.0,
        val.sin(),
        0.,
        1.,
        0.,
        -val.sin(),
        0.,
        val.cos(),
    )
}

pub fn get_rot_z(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(
        val.cos(),
        -val.sin(),
        0.,
        val.sin(),
        val.cos(),
        0.,
        0.,
        0.,
        1.,
    )
}

pub fn get_neg_rot_x(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_x(-val)
}

pub fn get_neg_rot_y(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_y(-val)
}

pub fn get_neg_rot_z(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_z(-val)
}

pub fn get_quat_x(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(val, 0., 0.)
}

pub fn get_quat_y(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(0., val, 0.)
}

pub fn get_quat_z(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(0., 0., val)
}

pub fn get_neg_quat_x(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_x(-val)
}

pub fn get_neg_quat_y(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_y(-val)
}

pub fn get_neg_quat_z(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_z(-val)
}

pub fn euler_triple_to_3x3(t: &[f64]) -> nalgebra::Matrix3<f64> {
    let xm = get_rot_x(t[0]);
    let ym = get_rot_y(t[1]);
    let zm = get_rot_z(t[2]);

    let zy = zm * ym;
    zy * xm
}
