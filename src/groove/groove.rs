use crate::groove::objective_master::ObjectiveMaster;
use crate::groove::vars::RelaxedIKVars;
use optimization_engine::{constraints::*, panoc::*, *};

use crate::groove::groove::core::SolverStatus;
pub struct OptimizationEngineOpen {
    _dim: usize,
    cache: PANOCCache,
}
impl OptimizationEngineOpen {
    pub fn new(dim: usize) -> Self {
        let cache = PANOCCache::new(dim, 1e-14, 10);
        OptimizationEngineOpen { _dim: dim, cache }
    }

    pub fn optimize(
        &mut self,
        x: &mut [f64],
        v: &RelaxedIKVars,
        om: &ObjectiveMaster,
        max_iter: usize,
    ) -> Result<SolverStatus, SolverError> {
        let df = move |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let (_, my_grad) = om.gradient(u, v);
            grad[..my_grad.len()].copy_from_slice(&my_grad[..]);
            Ok(())
        };

        let f = move |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = om.call(u, v);
            Ok(())
        };
        // let bounds = NoConstraints::new();
        let bounds = Rectangle::new(
            Option::from(v.robot.lower_joint_limits.as_slice()),
            Option::from(v.robot.upper_joint_limits.as_slice()),
        );

        /* PROBLEM STATEMENT */
        let problem = Problem::new(&bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut self.cache)
            .with_max_iter(max_iter)
            .with_tolerance(1e-5);
        // let mut panoc = PANOCOptimizer::new(problem, &mut self.cache);

        // Invoke the solver

        panoc.solve(x)
        // println!("Panoc status: {:#?}", status);
        // println!("Panoc solution: {:#?}", x);
    }


    
}

