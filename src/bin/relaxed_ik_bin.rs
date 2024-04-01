extern crate relaxed_ik_lib;
use relaxed_ik_lib::relaxed_ik;
use nalgebra::Vector3;

fn main() {
    // initialize relaxed ik
    let default_path_to_setting =   "configs/settings.yaml";
    let mut relaxed_ik = relaxed_ik::RelaxedIK::load_settings(default_path_to_setting);

    for _ in 0..10{
        for j in 0..relaxed_ik.vars.robot.num_chains {
            // gradually move along the y axis
            relaxed_ik.vars.goal_positions[j] += Vector3::new(0.0, 0.01, 0.0);
        }
        let x = relaxed_ik.solve();
        println!("Joint solutions: {:?}", x);
    }
}
