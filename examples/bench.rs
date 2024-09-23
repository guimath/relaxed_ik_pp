use clap::Parser;
use relaxed_ik_lib::relaxed_ik_wrapper::RelaxedWrapper;
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
    let mut rik = RelaxedWrapper::new(args.settings.to_str().unwrap());
    let objectives = rik.om.objectives;
    const TRY_NUM:i32 = NUM_PER_JOINT.pow(6);
    let call_per_ik :f64 = TRY_NUM as f64 / 67600.0 ;
    println!("Number of calls: {:} - calls per point in 260x260 : {:.2}", TRY_NUM, call_per_ik);
   
    let mut x = [0.0f64;6];
    rik.om.objectives = vec![];
    rik.om.weight_priors = vec![];
    macro_rules! scan_space_rec {
        () => ({
            rik.om.call(&x, &rik.vars);
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
    for _ in 0..loops_to_average {
        let t1 = time::Instant::now();
        scan_space_rec!(0,1,2,3,4,5);
        base_line += time::Instant::now() - t1;
    }
    base_line /= loops_to_average;

    rik.om.weight_priors = vec![1.0];
    for obj in objectives {
        rik.om.objectives = vec![obj];
        let t1 = time::Instant::now();
        scan_space_rec!(0,1,2,3,4,5);
        let elapsed = time::Instant::now() - t1;
        println!("{:?}", elapsed.clamp(base_line, time::Duration::from_secs(2000))-base_line);

    }
}
