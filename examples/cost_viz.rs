
#![allow(unused_imports)]
use relaxed_ik_lib::groove::objective::{groove_loss, swamp_groove_loss, swamp_loss};
use plotters::prelude::*;

use yaml_rust::{YamlLoader, Yaml};
use std::fs::File;
use std::io::prelude::*;
use std::path::PathBuf;
use clap::Parser;
/// args
#[derive(Parser)]
struct Cli {
    /// Specify title
    #[arg(short, long)]
    settings: Option<PathBuf>,
}

// fn test_func(x:f64) -> f64 {
//     let bound = 0.1;
//     swamp_groove_loss(x, 0.0, -bound, bound, bound*2.0, 1.0, 0.01, 50.0, 20)
//     groove_loss(x, 0., 2, 0.1, 10.0, 2)
// } 
fn yaml_to_vec(yaml_vec: Yaml) -> Option<Vec<f64>> {
    match yaml_vec.as_vec() {
        Some(v) => 
            Some(v
            .iter()
            .map(|yaml| yaml.as_f64().unwrap()) 
            .collect()),
        None => None,
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Cli::parse();
    let conf_file = args.settings.unwrap_or(PathBuf::from("examples/cost_configs/groove.yaml"));
    let mut pic_file = conf_file.clone();
    pic_file.set_extension("png");
    let mut file = File::open(conf_file).unwrap();
    let mut contents = String::new();
    let _res = file.read_to_string(&mut contents).unwrap();
    let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
    let settings = &docs[0];


    let mut t: f64 = 0.0;
    let mut d: i32 = 0;
    let mut c: f64 = 0.0;
    let mut f: f64 = 0.0;
    let mut g: i32 = 0;
    let mut l_bound: f64 = 0.0;
    let mut u_bound: f64 = 0.0;
    let mut f1: f64 = 0.0;
    let mut f2: f64 = 0.0;
    let mut f3: f64 = 0.0;
    let mut p1: i32 = 0;
    let mut g_s: f64 = 0.0;
    let func_type = settings["func_type"].as_str().expect("func_type needs to be specified");
    let title: String;
    
    match func_type {
        "groove" => {
            t = settings["t"].as_f64().expect("t should be specified as f64");
            d = settings["d"].as_i64().expect("d should be specified as i32") as i32;
            c = settings["c"].as_f64().expect("c should be specified as f64");
            f = settings["f"].as_f64().expect("f should be specified as f64");
            g = settings["g"].as_i64().expect("g should be specified as i32") as i32;
            title = format!("Groove t={t:.3} d={d:} c={c:.2} f={f:.1} g={g:}")
        }
        "swamp" => {
            l_bound = settings["l_bound"].as_f64().expect("l_bound should be specified as f64");
            u_bound = settings["u_bound"].as_f64().expect("u_bound should be specified as f64");
            f1 = settings["f1"].as_f64().expect("f1 should be specified as f64");
            f2 = settings["f2"].as_f64().expect("f2 should be specified as f64");
            p1 = settings["p1"].as_i64().expect("p1 should be specified as i32") as i32;
            title = format!("Swamp l_bound={l_bound:.1} u_bound={u_bound:.1} f1={f1:.1} f2={f2:.4} p1={p1:}")
        }
        "swamp_groove" => {
            g_s = settings["g_s"].as_f64().expect("g_s should be specified as f64");
            l_bound = settings["l_bound"].as_f64().expect("l_bound should be specified as f64");
            u_bound = settings["u_bound"].as_f64().expect("u_bound should be specified as f64");
            c = settings["c"].as_f64().expect("c should be specified as f64");
            f1 = settings["f1"].as_f64().expect("f1 should be specified as f64");
            f2 = settings["f2"].as_f64().expect("f2 should be specified as f64");
            f3 = settings["f3"].as_f64().expect("f3 should be specified as f64");
            p1 = settings["p1"].as_i64().expect("p1 should be specified as i32") as i32;
            title = format!("Swamp groove g_s={g_s:.1} l_bound={l_bound:.1} u_bound={u_bound:.1} c={c:.2} f1={f1:.1} f2={f2:.2} f3={f3:.1} p1={p1:}")
        }
        _ => {println!("func type not supported (only groove, swamp & swamp_groove) "); return Ok(())},
    };
    
    let test_func = |x:f64| {match func_type {
        "groove" => groove_loss(x, t, d, c, f, g),
        "swamp" => swamp_loss(x, l_bound, u_bound, f1, f2, p1),
        "swamp_groove" => swamp_groove_loss(x, g_s, l_bound, u_bound, c, f1, f2, f3, p1),
            _ => 0.0
    }};
    
    let x_zone = yaml_to_vec(settings["x_zone"].clone()).unwrap_or(vec![-0.5, 0.5]);
    let y_zone = yaml_to_vec(settings["y_zone"].clone()).unwrap_or(vec![-1.0, 50.0]); 
    let nb_points = settings["nb_points"].as_i64().unwrap_or(100);
    let x_scale= (x_zone[0]-x_zone[1]).abs();
    /* GRAPH */
    let root = BitMapBackend::new(pic_file.as_os_str(), (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 16).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(x_zone[0]..x_zone[1], y_zone[0]..y_zone[1])?;

    chart.configure_mesh().draw()?;

    chart
        .draw_series(LineSeries::new(
            (0..=nb_points).map(|x| x as f64 / (nb_points as f64 * x_scale) + x_zone[0]).map(|x| (x, test_func(x))),
            &BLUE,
        ))?
        .label(format!("{:?}", pic_file.file_stem().unwrap()));

    // chart
    //     .configure_series_labels()
    //     .background_style(&WHITE.mix(0.8))
    //     .border_style(&BLACK)
    //     .draw()?;

    root.present()?;
    println!("{:?}", pic_file.file_stem().unwrap());

    Ok(())
}
