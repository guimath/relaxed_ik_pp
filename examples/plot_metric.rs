use base64::{engine::general_purpose, Engine as _};
use clap::Parser;
use plotters::prelude::*;
use relaxed_ik_lib::errors::UnfeasibleTrajectoryPoint;
use relaxed_ik_lib::Error;
use savefile::prelude::*;
use scarlet::color::RGBColor;
use scarlet::colormap::{ColorMap, GradientColorMap};
use serde::Deserialize;
use std::fs::File;
use std::io::Read;
use std::path::PathBuf;

use svg2pdf::{ConversionOptions, PageOptions};

/// args
#[derive(Parser)]
struct Cli {
    /// Specify path to the data file
    #[arg(
        short,
        long,
        default_value = "ex_out/ur5/ik_Z_+0.3.bin",
        value_name = "FILE PATH"
    )]
    data_file: PathBuf,
    /// Save to svg instead of pdf
    #[arg(short, long, default_value_t = false)]
    svg: bool,
}

#[derive(Deserialize, Debug, Default)]
struct GraphConfig {
    x_zone: [f64; 2],
    y_zone: [f64; 2],
    range_scale: Option<[f64; 2]>,
}

fn main() {
    let args = Cli::parse();
    // TOML :
    let mut data_toml = args.data_file.clone();
    data_toml.set_extension("toml");
    let mut file = File::open(data_toml).unwrap();
    let mut contents = String::new();
    let _res = file.read_to_string(&mut contents).unwrap();
    let res: Result<GraphConfig, toml::de::Error> = toml::from_str(&contents);
    if let Err(e) = res {
        println!("{}", e);
        return;
    }
    let config = res.unwrap();
    // setup var
    let range = config.range_scale.unwrap_or([-246.2, -200.0]);
    let min_val = range[0];
    let max_val = range[1];
    let delta_val = max_val - min_val;
    let mut true_min = f64::INFINITY;
    let mut true_max = -f64::INFINITY;

    let red = RGBColor::from_hex_code("#ff0000").unwrap();
    let green = RGBColor::from_hex_code("#00ff00").unwrap();
    let cmap = GradientColorMap::new_linear(green, red);
    let mut val_to_color = |val: f64| {
        if true_max < val {
            true_max = val
        };
        if true_min > val {
            true_min = val
        };
        let h = (val.clamp(min_val, max_val) - min_val) / delta_val;
        let color = cmap.transform_single(h);
        plotters::prelude::RGBColor(color.int_r(), color.int_g(), color.int_b()).to_rgba()
    };

    let mut res_to_color = |res: Result<f64, Error>| match res {
        Err(Error::OutOfRange) => WHITE.to_rgba(),
        Err(Error::Cost) => CYAN.to_rgba(),
        Err(Error::NotFiniteComputation) => CYAN.to_rgba(),
        Err(Error::Collision {
            point: p,
            collision_link_names: _,
        }) => match p {
            UnfeasibleTrajectoryPoint::Start => BLACK.to_rgba(),
            UnfeasibleTrajectoryPoint::Goal => MAGENTA.to_rgba(),
            UnfeasibleTrajectoryPoint::WayPoint => MAGENTA.to_rgba(),
        },
        Err(_) => BLUE.to_rgba(),
        Ok(val) => val_to_color(val),
    };

    let data: Vec<Vec<Result<f64, Error>>> = load_file(args.data_file.clone(), 0).unwrap();
    let i_max = data.len();
    let j_max = data[0].len();
    let start_x = config.x_zone[0];
    let start_y = config.y_zone[0];
    let step_x = (config.x_zone[1] - start_x) / i_max as f64;
    let step_y = (config.y_zone[1] - start_y) / i_max as f64;

    let mut pic_file = PathBuf::from(args.data_file.clone());
    pic_file.pop(); //removing data folder from path
    pic_file.set_file_name(args.data_file.file_name().unwrap());
    if args.svg {
        pic_file.set_extension("svg");
    } else {
        pic_file.set_extension("pdf");
    }
    println!("Graph will be saved to {:#?}", pic_file.as_os_str());

    // Actual heatmap is PNG to avoid to complicated svg
    let mut backend = BitMapBackend::new("ex_out/temp.png", (i_max as u32, j_max as u32));
    for i in 0..i_max {
        for j in 0..j_max {
            let col = res_to_color(data[j][i].clone());
            backend
                .draw_pixel((i as i32, j as i32), col.to_backend_color())
                .unwrap();
        }
    }
    backend.present().unwrap();
    println!("true min {true_min} true max {true_max}");

    // SVG Creation
    let mut svg_content = String::new();
    {
        let root = SVGBackend::with_string(&mut svg_content, (1120, 1000)).into_drawing_area();
        // root.fill(&WHITE).unwrap();
        let (upper, lower) = root.split_horizontally(1020);
        // top margin = 10
        // x label = 60
        // bottom margin = 10
        // delta = 80 -> size of img 1000 - 80 = 920

        // left margin = 10
        // y label = 80
        // right margin = 10
        // scale = 100
        // delta = 200 -> 920 + 200 = 1220
        // Axis
        let mut chart = ChartBuilder::on(&upper)
            // .caption("Graph", ("sans-serif", 40))
            .margin(10)
            .top_x_label_area_size(60)
            .y_label_area_size(80)
            .build_cartesian_2d(0i32..(i_max as i32), (j_max as i32)..0i32)
            .unwrap();
        chart
            .configure_mesh()
            .x_desc("X (m)")
            .y_desc("Y (m)")
            .x_labels(12)
            .y_labels(12)
            .max_light_lines(4)
            // TODO make params
            .x_label_formatter(&|r| format!("{:.2}", start_x + step_x * (*r as f64)))
            .y_label_formatter(&|r| format!("{:.2}", start_y + step_y * (*r as f64)))
            .disable_x_mesh()
            .disable_y_mesh()
            .label_style(("sans-serif", 20))
            .draw()
            .unwrap();

        // SCALE
        const POINTS_NUM: usize = 50;
        let mut chart = ChartBuilder::on(&lower)
            .y_label_area_size(80)
            .margin_bottom(10)
            .margin_top(40 + 10 + 40)
            .build_cartesian_2d(0i32..1i32, 0i32..POINTS_NUM as i32)
            .unwrap();

        chart
            .configure_mesh()
            .y_labels(10)
            .y_label_formatter(&|r| {
                format!(
                    "{:.2}",
                    min_val + delta_val / (POINTS_NUM as f64) * (*r as f64)
                )
            })
            .max_light_lines(4)
            .x_label_offset(35)
            .y_label_offset(0)
            .disable_x_mesh()
            .disable_y_mesh()
            .disable_x_axis()
            .label_style(("sans-serif", 20))
            .draw()
            .unwrap();

        let mut matrix = [[WHITE.to_rgba(); 1]; POINTS_NUM];
        for i in 0..POINTS_NUM {
            let color = cmap.transform_single((i as f64) / (POINTS_NUM as f64));
            matrix[i][0] =
                plotters::prelude::RGBColor(color.int_r(), color.int_g(), color.int_b()).to_rgba();
        }

        chart
            .draw_series(
                matrix
                    .iter()
                    .zip(0..)
                    .flat_map(|(l, y)| l.iter().zip(0..).map(move |(v, x)| (x, y, v)))
                    .map(|(x, y, v)| Rectangle::new([(x, y), (x + 1, y + 1)], v.filled())),
            )
            .unwrap();

        let _ = root.present();
    }

    let img = image::open("ex_out/temp.png").unwrap().to_rgba8();
    // let img: ImageBuffer<image::Rgba<u8>, Vec<u8>> = ImageBuffer::from_raw(400, 400, img_buffer).unwrap();
    let mut image_data: Vec<u8> = Vec::new();
    let _ = img.write_to(
        &mut std::io::Cursor::new(&mut image_data),
        image::ImageOutputFormat::Png,
    );
    // println!("{} vs {}", image_data.len(), raw_image_data.len());
    let res_base64 = general_purpose::STANDARD.encode(image_data.clone());
    // println!("{} - {} - {} - {}", image_data[0], image_data[1], image_data[2], image_data[3]);
    let img_tag = format!(
        r#"<image x="90" y="70" width="920" height="920" href="data:image/png;base64,{}" /></svg>"#,
        res_base64
    );
    svg_content = svg_content[..svg_content.len() - 7].to_string(); // removing /svg tag
    svg_content.push_str(&img_tag);
    if args.svg {
        std::fs::write(pic_file, svg_content).unwrap();
        return;
    }

    let mut options = svg2pdf::usvg::Options::default();
    options.fontdb_mut().load_system_fonts();
    let tree = svg2pdf::usvg::Tree::from_str(&svg_content, &options).unwrap();

    let pdf = svg2pdf::to_pdf(&tree, ConversionOptions::default(), PageOptions::default()).unwrap();
    std::fs::write(pic_file, pdf).unwrap();
}
