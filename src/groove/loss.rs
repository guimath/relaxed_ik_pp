
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
    pub g: i32
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
    pub p1: i32
}


/// * `g` center of the groove
/// * `l_bound` lower bound
/// * `u_bound` upper bound
/// * `c` standard deviation of groove (width)
/// * `f1` depth of swamp
/// * `f2` penalty coefficient outisde of swamp
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
    /// penalty coefficient outisde of swamp
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
    Groove(GrooveParams)
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub enum SwampType {
    Swamp(SwampParams),
}

#[inline]
pub fn swamp_loss(x_val: f64, p: SwampParams) -> f64 {
    let x = (2.0 * x_val - p.l_bound - p.u_bound) / (p.u_bound - p.l_bound);
    let b = (-1.0 / 0.05_f64.ln()).powf(1.0 / p.p1 as f64);
    (p.f1 + p.f2 * x.powi(2)) * (1.0 - (-(x / b).powi(p.p1)).exp()) - 1.0
}

#[inline]
pub fn swamp_groove_loss(x_val: f64, p: SwampGrooveParams) -> f64 {
    let x = (2.0 * x_val - p.l_bound - p.u_bound) / (p.u_bound - p.l_bound);
    let b = (-1.0 / 0.05_f64.ln()).powf(1.0 / p.p1 as f64);
    -p.f1 * ((-(x_val - p.g).powi(2)) / (2.0 * p.c.powi(2))).exp()
        + p.f2 * (x_val - p.g).powi(2)
        + p.f3 * (1.0 - (-(x / b).powi(p.p1)).exp())
}

#[inline]
pub fn groove_loss(x_val: f64, p: GrooveParams) -> f64 {
    -((-(x_val - p.t).powi(p.d)) / (2.0 * p.c.powi(2))).exp() + p.f * (x_val - p.t).powi(p.g)
}

pub fn get_loss_desc(function:FuncType) -> String{
    let s = format!("{:#?}",function);
    let l : Vec<&str> = s.lines().collect();
    let lines: Vec<&str> = l.iter().map(|s| s.trim()).collect();
    let core = lines[2..lines.len() - 2].join(" ");
    lines[0].to_string() + &core[..core.len()-1] + ")"
}

#[inline]
pub fn get_loss_func(function:FuncType) -> Box<dyn Fn(f64) -> f64> {
    match function {
        FuncType::Groove(p)=>{
            Box::new(move |x:f64| groove_loss(x, p))
        },
        FuncType::Swamp(p)=>{
            Box::new(move |x:f64| swamp_loss(x, p))
        },
        FuncType::SwampGroove(p)=>{
            Box::new(move |x:f64| swamp_groove_loss(x, p))
        }
    }
}


