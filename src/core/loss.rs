use serde::Deserialize;

/// * `t` center of the groove
/// * `d` degree
/// * `c` standard deviation of groove (width)
/// * `f` penalty coefficient outside of groove
/// * `g` polynomial degree
#[derive(Deserialize, Debug, Clone, Copy)]
pub struct GrooveParams {
    /// center of the groove
    pub t: f64,
    /// degree
    pub d: i32,
    /// standard deviation of groove (width)
    pub c: f64,
    /// penalty coefficient outside of groove
    pub f: f64,
    /// polynomial degree
    pub g: i32,
}

///  * `l_bound` lower bound
///  * `u_bound` upper bound
///  * `f1` height of walls
///  * `f2` penalty coefficient outside of swamp
///  * `p1` sharpness of walls
#[derive(Deserialize, Debug, Clone, Copy)]
pub struct SwampParams {
    /// lower bound
    pub l_bound: f64,
    /// upper bound
    pub u_bound: f64,
    /// height of walls
    pub f1: f64,
    /// penalty coefficient outside of swamp
    pub f2: f64,
    /// sharpness of walls
    pub p1: i32,
}

/// * `g` center of the groove
/// * `l_bound` lower bound
/// * `u_bound` upper bound
/// * `c` standard deviation of groove (width)
/// * `f1` depth of swamp
/// * `f2` penalty coefficient outside of swamp
/// * `f3` height of walls
/// * `p1` sharpness of walls
#[derive(Deserialize, Debug, Clone, Copy)]
pub struct SwampGrooveParams {
    /// center of the groove
    pub g: f64,
    /// lower bound
    pub l_bound: f64,
    /// upper bound
    pub u_bound: f64,
    /// standard deviation of groove (width)
    pub c: f64,
    /// depth of swamp
    pub f1: f64,
    /// penalty coefficient outside of swamp
    pub f2: f64,
    /// height of walls
    pub f3: f64,
    /// sharpness of walls
    pub p1: i32,
}

/// Enum of all three loss functions
#[derive(Deserialize, Debug, Clone, Copy)]
pub enum FuncType {
    Swamp(SwampParams),
    SwampGroove(SwampGrooveParams),
    Groove(GrooveParams),
}

impl FuncType {
    pub fn into_inner(self) -> Box<dyn LossFunction> {
        match self {
            FuncType::Swamp(params) => Box::new(params),
            FuncType::SwampGroove(params) => Box::new(params),
            FuncType::Groove(params) => Box::new(params),
        }
    }
}

impl LossFunction for FuncType {
    fn compute(&self, x: f64) -> f64 {
        match self {
            FuncType::Swamp(params) => params.compute(x),
            FuncType::SwampGroove(params) => params.compute(x),
            FuncType::Groove(params) => params.compute(x),
        }
    }
    fn recap(&self) -> String {
        match self {
            FuncType::Swamp(params) => params.recap(),
            FuncType::SwampGroove(params) => params.recap(),
            FuncType::Groove(params) => params.recap(),
        }
    }
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub enum SwampType {
    Swamp(SwampParams),
}

pub trait LossFunction {
    fn compute(&self, x: f64) -> f64;
    fn recap(&self) -> String;
}

impl LossFunction for SwampParams {
    fn compute(&self, x: f64) -> f64 {
        let x = (2.0 * x - self.l_bound - self.u_bound) / (self.u_bound - self.l_bound);
        let b = (-1.0 / 0.05_f64.ln()).powf(1.0 / self.p1 as f64);
        (self.f1 + self.f2 * x.powi(2)) * (1.0 - (-(x / b).powi(self.p1)).exp()) - 1.0
    }
    fn recap(&self) -> String {
        format!("{:?}", self)
    }
}

impl LossFunction for SwampGrooveParams {
    fn compute(&self, x: f64) -> f64 {
        let x = (2.0 * x - self.l_bound - self.u_bound) / (self.u_bound - self.l_bound);
        let b = (-1.0 / 0.05_f64.ln()).powf(1.0 / self.p1 as f64);
        -self.f1 * ((-(x - self.g).powi(2)) / (2.0 * self.c.powi(2))).exp()
            + self.f2 * (x - self.g).powi(2)
            + self.f3 * (1.0 - (-(x / b).powi(self.p1)).exp())
    }
    fn recap(&self) -> String {
        format!("{:?}", self)
    }
}

impl LossFunction for GrooveParams {
    fn compute(&self, x: f64) -> f64 {
        -((-(x - self.t).powi(self.d)) / (2.0 * self.c.powi(2))).exp()
            + self.f * (x - self.t).powi(self.g)
    }
    fn recap(&self) -> String {
        format!("{:?}", self)
    }
}
