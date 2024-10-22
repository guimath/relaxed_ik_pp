use plotters::prelude::*;
use serde::Deserialize;

use clap::Parser;
use relaxed_ik_lib::groove::{loss, loss::FuncType};
use std::fs::File;
use std::io::prelude::*;
use std::path::PathBuf;
/// args
#[derive(Parser)]
struct Cli {
    /// Specify path to the cost config file.
    #[arg(
        short,
        long,
        default_value = "examples/cost_configs/example.toml",
        value_name = "FILE PATH"
    )]
    settings: PathBuf,
}
#[derive(Deserialize, Debug, Default)]
struct GraphOptions {
    title: Option<String>,
    x_zone: Option<[f64; 2]>,
    y_zone: Option<[f64; 2]>,
    nb_points: Option<u64>,
    graph_size: Option<[u32; 2]>,
}

#[derive(Deserialize)]
struct Config {
    graph_options: Option<GraphOptions>,
    function1: FuncType,
    function2: Option<FuncType>,
    function3: Option<FuncType>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args: Cli = Cli::parse();
    let mut pic_file = args.settings.clone();
    pic_file.set_extension("svg");
    let mut file = File::open(args.settings.clone()).unwrap();
    let mut contents = String::new();
    let _res = file.read_to_string(&mut contents).unwrap();

    let res: Result<Config, toml::de::Error> = toml::from_str(&contents);
    if let Err(e) = res {
        println!("{}", e);
        return Ok(());
    }
    let config = res.unwrap();
    const COLORS: [RGBColor; 3] = [RED, BLACK, BLUE];
    let mut desc: Vec<String> = vec![];
    let mut test_func: Vec<Box<dyn Fn(f64) -> f64>> = vec![];

    desc.push(loss::get_loss_desc(config.function1));
    test_func.push(loss::get_loss_func(config.function1));

    if let Some(func) = config.function2 {
        desc.push(loss::get_loss_desc(func));
        test_func.push(loss::get_loss_func(func));
    }
    if let Some(func) = config.function3 {
        desc.push(loss::get_loss_desc(func));
        test_func.push(loss::get_loss_func(func));
    }

    let nb_plot = desc.len();

    let graph_opt = config.graph_options.unwrap_or(Default::default());
    let nb_points = graph_opt.nb_points.unwrap_or(200);
    let x_zone = graph_opt.x_zone.unwrap_or([-0.5, 0.5]);
    let y_zone = graph_opt.y_zone.unwrap_or([-1.0, 10.0]);
    let x_scale = (x_zone[0] - x_zone[1]).abs();
    let graph_size = graph_opt.graph_size.unwrap_or([1920, 1080]);
    let title = graph_opt.title.unwrap_or(format!(
        "{}",
        args.settings.file_stem().unwrap().to_str().unwrap()
    ));
    let root = SVGBackend::new(pic_file.as_os_str(), graph_size.into()).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 28).into_font())
        .margin(5)
        .margin_right(20)
        .x_label_area_size(30)
        .y_label_area_size(50)
        .build_cartesian_2d(x_zone[0]..x_zone[1], y_zone[0]..y_zone[1])?;

    chart
        .configure_mesh()
        .label_style(("sans-serif", 20))
        .draw()?;

    for i in 0..nb_plot {
        chart
            .draw_series(LineSeries::new(
                (0..=nb_points)
                    .map(|x| x as f64 * (x_scale / nb_points as f64) + x_zone[0])
                    .map(|x| (x, test_func[i](x))),
                &COLORS[i],
            ))?
            .label(format!("{}", desc[i]))
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], COLORS[i].filled()));
    }

    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .label_font(("sans-serif", 28))
        .draw()?;

    root.present()?;
    println!("Saved to {:?}", pic_file.as_os_str());

    Ok(())
}
