use nalgebra::{UnitQuaternion, Vector3};

pub type Pose = (Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>);
pub type SinglePose = (Vector3<f64>, UnitQuaternion<f64>);
