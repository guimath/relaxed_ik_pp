use plotters::prelude::*;

const OUT_FILE_NAME: &str = "ex_out/legend.svg";

fn add_legend(
    chart: &mut ChartContext<
        '_,
        SVGBackend<'_>,
        Cartesian2d<plotters::coord::types::RangedCoordf64, plotters::coord::types::RangedCoordf64>,
    >,
    title: &str,
    color: RGBColor,
) {
    chart
        .draw_series(LineSeries::new(vec![(0.1, 0.1)], WHITE))
        .unwrap()
        .label(title)
        .legend(move |(x, y)| Rectangle::new([(x, y - 10), (x + 20, y + 10)], color.filled()));
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = SVGBackend::new(OUT_FILE_NAME, (400, 250)).into_drawing_area();
    root.fill(&RGBColor(200, 200, 200))?;

    let mut chart = ChartBuilder::on(&root)
        .margin(0)
        .set_label_area_size(LabelAreaPosition::Left, 0)
        .set_label_area_size(LabelAreaPosition::Bottom, 0)
        .build_cartesian_2d(-10f64..10f64, -10f64..10f64)?;

    add_legend(&mut chart, "IK not converged", CYAN);
    add_legend(&mut chart, "Collision at start", BLACK);
    add_legend(&mut chart, "Collision at end", MAGENTA);
    add_legend(&mut chart, "Path planning failed", RED);
    add_legend(&mut chart, "Grasp pose reached", GREEN);
    // FontFamily::Serif;
    chart
        .configure_series_labels()
        .background_style(RGBColor(200, 200, 200).filled())
        .label_font(("sans-serif", 40))
        .draw()?;

    // To avoid the IO failure being ignored silently, we manually call the present function
    root.present().unwrap();
    println!("Result has been saved to {}", OUT_FILE_NAME);

    Ok(())
}
