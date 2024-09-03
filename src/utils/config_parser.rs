use log::warn;
use std::fmt::Debug;
use std::fs::File;
use std::io::prelude::*;
use std::path::{Path, PathBuf};
use yaml_rust::{Yaml, YamlLoader};
#[derive(Clone, Debug)]
pub struct Config {
    pub robot_urdf_path: PathBuf,
    pub obstacles_urdf_path: Option<PathBuf>,
    pub used_links: Vec<String>,
    pub base_links: Vec<String>,
    pub ee_links: Vec<String>,
    pub starting_config: Vec<f64>,
    pub package: Option<String>,
    pub approach_dist: f64,
    pub cost_threshold: f64,
}

fn yaml_to_vec(yaml_vec: Yaml, err_msg: &str) -> Vec<String> {
    yaml_vec
        .as_vec()
        .unwrap()
        .iter()
        .map(|yaml| yaml.as_str().expect(err_msg).to_string())
        .collect()
}

impl Config {
    pub fn from_settings_file<P: AsRef<Path> + Clone>(path_to_setting: P) -> Self {
        let mut file = File::open(path_to_setting.clone()).unwrap();
        let mut contents = String::new();
        let _res = file.read_to_string(&mut contents).unwrap();
        let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
        let settings = &docs[0];
        let mut robot_urdf_path = PathBuf::from(path_to_setting.as_ref());
        robot_urdf_path.set_file_name("urdfs");

        let obstacles_urdf_path = match settings["obstacles"].as_str() {
            Some(file_name) => {
                let mut path = robot_urdf_path.clone();
                path.push(file_name);
                Some(path)
            }
            None => None,
        };

        robot_urdf_path.push(settings["urdf"].as_str().unwrap());

        let base_links = yaml_to_vec(settings["base_links"].clone(), "base_links parsing error");
        let ee_links = yaml_to_vec(settings["ee_links"].clone(), "ee_links parsing error");
        let used_links = yaml_to_vec(settings["used_links"].clone(), "used_links parsing error");
        let dof = used_links.len();
        let mut starting_config = Vec::new();
        if settings["starting_config"].is_badvalue() {
            warn!("No starting config provided, using all zeros");
            starting_config = vec![0.0; dof];
        } else {
            let starting_config_arr = settings["starting_config"].as_vec().unwrap();
            let arr_len = starting_config_arr.len();
            for start_conf_yaml in starting_config_arr.iter() {
                starting_config.push(start_conf_yaml.as_f64().unwrap());
            }
            if arr_len < dof {
                warn!(
                    "Starting config not same size as dof ({:?} vs {:?}), padding with zeros",
                    arr_len, dof
                );
                starting_config.extend(vec![0.0; dof-arr_len]);
            }
        }
        let package = settings["base_links"].as_str().map(|p| p.to_string());
        let approach_dist = settings["approach_dist"].as_f64().unwrap_or(0.03);
        let cost_threshold = settings["cost_threshold"].as_f64().unwrap_or(-50.0);

        Self {
            robot_urdf_path,
            obstacles_urdf_path,
            used_links,
            base_links,
            ee_links,
            starting_config,
            package,
            approach_dist,
            cost_threshold,
        }
    }
}
