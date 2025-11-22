use crate::core::{objective_master::ObjectiveMaster, vars::RelaxedIKVars};
use optimization_engine::{constraints::*, panoc::*, *};

use crate::core::groove::core::SolverStatus;
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
            let (_, my_grad) = om.optimized_grad(u, v);
            grad[..my_grad.len()].copy_from_slice(&my_grad[..]);
            Ok(())
        };

        let f = move |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = om.call(u, v);
            Ok(())
        };
        // let bounds = NoConstraints::new();
        let (lower_bounds, upper_bounds): (Vec<f64>, Vec<f64>) = v
            .robot
            .joint_limits
            .iter()
            .map(|limits| (limits.lower_bound, limits.upper_bound))
            .unzip();
        let bounds: Rectangle<'_> = Rectangle::new(
            Option::from(lower_bounds.as_slice()),
            Option::from(upper_bounds.as_slice()),
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
