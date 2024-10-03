use clap::Parser;
use relaxed_ik_lib::relaxed_ik::RelaxedIK;
use std::{path::PathBuf, time};

/// args
#[derive(Parser)]
struct Cli {
    /// Specify path to the settings file.
    #[arg(
        short,
        long,
        default_value = "configs/ur5.toml",
        value_name = "FILE PATH"
    )]
    settings: PathBuf,
}

const NUM_PER_JOINT:i32 = 12; 
const STEP:f64 = 1.0 / NUM_PER_JOINT as f64;
#[inline]
fn to_joint_val(x:i32) -> f64{
    0.5 - STEP*x as f64
}

fn main() {
    env_logger::init();
    let args = Cli::parse();
    // let conf = Config::from_settings_file(args.settings.clone());
    // rik init
    let mut rik = RelaxedIK::new(args.settings.to_str().unwrap());
    let rik2 = RelaxedIK::new(args.settings.to_str().unwrap());
    let objectives = rik.om.objectives;
    const TRY_NUM:i32 = NUM_PER_JOINT.pow(6);
    let call_per_ik :f64 = TRY_NUM as f64 / 67600.0 ;
    println!("Number of calls: {:} - calls per point in 260x260 : {:.2}", TRY_NUM, call_per_ik);
   
    let mut x = [0.0f64;6];
    rik.om.objectives = vec![];
    rik.om.weight_priors = vec![];
    macro_rules! scan_space_rec {
        () => ({
            // rik.om.optimized_grad(&x, &rik.vars);
            rik.om.gradient(&x, &rik.vars);
        });
        ($current:expr $(, $next:expr)*) => ({
            for x0 in 0..NUM_PER_JOINT {
                x[$current] = to_joint_val(x0);
                scan_space_rec!($($next),*);
            }
        });
    }
    let mut base_line = time::Duration::from_micros(0);
    let loops_to_average = 10;
    for i in 0..loops_to_average {
        let t1 = time::Instant::now();
        scan_space_rec!(0,1,2,3,4,5);
        base_line += time::Instant::now() - t1;
        println!("{:}", (base_line/(i+1)).as_millis())
    }
    base_line /= loops_to_average;
    println!("Global ; No baseline");
    println!("{:} ; 0", base_line.as_millis());

    rik.om.weight_priors = vec![1.0];
    let mut i = objectives.len()-8;
    for obj in objectives {
        rik.om.objectives = vec![obj];
        let t1 = time::Instant::now();
        scan_space_rec!(0,1,2,3,4,5);
        let elapsed = time::Instant::now() - t1;
        let sep = elapsed.clamp(base_line, time::Duration::from_secs(2000))-base_line;
        println!("{:} ; {:}", elapsed.as_millis(), sep.as_millis());
        i -= 1;
        if i == 0 {break}
    }

    rik.om.objectives = rik2.om.objectives;
    rik.om.weight_priors = rik2.om.weight_priors;
    let t1 = time::Instant::now();
    scan_space_rec!(0,1,2,3,4,5);
    let elapsed = time::Instant::now() - t1;
    let sep = elapsed.clamp(base_line, time::Duration::from_secs(2000))-base_line;
    println!("{:} ; {:}", elapsed.as_millis(), sep.as_millis());

}
// Results : Optimized gradient much faster baseline (partial frame calc improvements)
//(all times in ms) 
//                         |__gradient_finite_diff | optimized_grad        |
//                         | Global | No baseline  | Global |  No baseline |                                                           
// Base line               | 4979   | 0            | 2952   | 0            |                       
// MatchEEPosiDoF (Z)      | 5725   | 745          | 3812   | 860          |                                       
// MatchEEPosiDoF (X)      | 5655   | 675          | 3876   | 923          |                                       
// MatchEEPosiDoF (Y)      | 5672   | 692          | 3846   | 893          |                                       
// HorizontalArm           | 5517   | 537          | 3685   | 733          |                                   
// HorizontalGripper       | 6667   | 1688         | 4870   | 1917         |                                       
// EachJointLimits (0)     | 5688   | 708          | 3755   | 803          |                                       
// EachJointLimits (1)     | 5778   | 798          | 3801   | 848          |                                       
// EachJointLimits (2)     | 5832   | 853          | 3864   | 911          |                                       
// EachJointLimits (3)     | 5801   | 822          | 3724   | 771          |                                       
// EachJointLimits (4)     | 6351   | 1372         | 3864   | 911          |                                       
// EachJointLimits (5)     | 6225   | 1245         | 3719   | 767          |                                       
// MinimizeVelocity        | 6065   | 1085         | 3555   | 602          |                                   
// MinimizeAcceleration    | 6136   | 1157         | 3762   | 809          |                                       
// MinimizeJerk            | 6046   | 1067         | 3844   | 892          |                               
// MaximizeManipulability  | 25684  | 20704        | 23025  | 20072        |                                           
// SelfCollision (0 – 2)   | 7713   | 2734         | 5576   | 2623         |                                           
// SelfCollision (0 – 3)   | 7945   | 2966         | 5740   | 2787         |                                           
// All                     | 54179  | 49199        | 51665  | 48712        |                       
