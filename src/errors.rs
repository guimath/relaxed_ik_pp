use savefile_derive::Savefile;
use thiserror::Error;



pub fn point_from_str(point:&str) -> UnfeasibleTrajectoryPoint{
    match point {
        "Start"=> UnfeasibleTrajectoryPoint::Start,
        "WayPoint"=> UnfeasibleTrajectoryPoint::WayPoint,
        "Goal"=> UnfeasibleTrajectoryPoint::Goal,
        _ => panic!("Not a point")
    }
}

#[derive(Debug, Clone, Savefile)]
pub enum UnfeasibleTrajectoryPoint {
    Start,
    WayPoint,
    Goal,
}
/// Error for `openrr_planner`
#[derive(Debug, Error, Clone, Savefile)]
#[non_exhaustive]
pub enum Error {
    #[error("[IK] Cost or gradient function cannot be evaluated")]
    Cost,
    #[error("[IK] Computation failed and NaN/Infinite value was obtained")]
    NotFiniteComputation,
    #[error("[IK] Target out of range")]
    OutOfRange,
    #[error("Path not found")]
    PathPlanFail,
    #[error("Collision error: {collision_link_names:?} is colliding ({point:?})")]
    Collision {
        point: UnfeasibleTrajectoryPoint,
        collision_link_names: Vec<String>,
    },
    #[error("Self Collision error: {collision_link_names:?} is colliding ({point:?})")]
    SelfCollision {
        point: UnfeasibleTrajectoryPoint,
        collision_link_names: Vec<(String, String)>,
    },
    #[error("{}", error)]
    Other { error: String },
}