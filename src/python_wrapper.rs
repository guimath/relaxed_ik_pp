use pyo3::prelude::*;
use crate::relaxed_ik::RelaxedIK;

#[pyclass]
struct RelaxedWrapper {
    rik: RelaxedIK,
}

#[pymethods]
impl RelaxedWrapper {
    #[new]
    pub fn new(path_to_setting: &str) -> Self {
        Self{rik:RelaxedIK::new(path_to_setting)}
    }

    pub fn grip(&mut self, pos_goals: [f64; 3]) -> Vec<Vec<f64>> {
        let (mut q1, q2, _) = self.rik.grip(pos_goals).unwrap();
        q1.extend(q2);
        q1
    }

    pub fn ik(&mut self, pos_goals:[f64; 3])-> Vec<f64> {
        self.rik.repeat_solve_ik(pos_goals).unwrap();
        self.rik.vars.xopt.clone()
    }

    pub fn reset(&mut self, x: Vec<f64>) {
        self.rik.reset(x.clone());
    }

    pub fn reset_origin(&mut self) {
        self.rik.reset_origin();
    }
}

#[pymodule]
fn relaxed_ik_lib(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();
    m.add_class::<RelaxedWrapper>()?;
    Ok(())
}