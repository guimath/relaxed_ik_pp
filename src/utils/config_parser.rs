
use yaml_rust::{YamlLoader, Yaml};
use std::fmt::Debug;
use std::fs::File;
use std::io::prelude::*;
use std::path::{Path, PathBuf};
use log::warn;
#[derive(Clone, Debug)]
pub struct Config
{
    pub robot_urdf_path : PathBuf,
    pub obstacles_urdf_path : Option<PathBuf>,
    pub used_links : Vec<String>,
    pub base_links : Vec<String>,
    pub ee_links : Vec<String>,
    pub starting_config: Vec<f64>,
    pub package: Option<String>,
    pub approach_dist: f64,
    pub cost_threshold : f64,
}


fn yaml_to_vec(yaml_vec: Yaml, err_msg: &str) -> Vec<String> {

    yaml_vec.as_vec()
        .unwrap()
        .iter()
        .map(|yaml| yaml.as_str().expect(err_msg).to_string()) 
        .collect() 
}


impl Config 
{
    pub fn from_settings_file<P: AsRef<Path> + Clone > (path_to_setting: P) -> Self{

        let mut file = File::open(path_to_setting.clone()).unwrap();
        let mut contents = String::new();
        let _res = file.read_to_string(&mut contents).unwrap();
        let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
        let settings = &docs[0];
        let mut robot_urdf_path = PathBuf::from(path_to_setting.as_ref());
        robot_urdf_path.set_file_name("urdfs".to_owned());
        
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
            for _ in 0..dof {
                starting_config.push(0.0);
            }
        } else {
            let starting_config_arr = settings["starting_config"].as_vec().unwrap();
            let arr_len = starting_config_arr.len();
            for i in 0..arr_len {
                starting_config.push(starting_config_arr[i].as_f64().unwrap());
            }
            if arr_len < dof {
                warn!("Starting config not same size as dof ({:?} vs {:?}), padding with zeros", arr_len, dof);
                for _ in arr_len..dof {
                    starting_config.push(0.0);
                }
            }
        }
        let package = match settings["base_links"].as_str(){
            Some(p) => Some(p.to_string()),
            None => None
        };
        let approach_dist = settings["approach_dist"].as_f64().unwrap_or(0.03);
        let cost_threshold = settings["cost_threshold"].as_f64().unwrap_or(-50.0);

        Self{
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