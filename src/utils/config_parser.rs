use std::collections::HashMap;
use std::fmt::Debug;
use std::fs::File;
use std::io::prelude::*;
use std::path::PathBuf;
use serde::Deserialize;
use crate::groove::objective_master::{ObjectivesConfig, ObjectivesConfigParse};
#[derive(Deserialize, Debug, Clone)]
pub struct UrdfPath {
    pub robot:PathBuf,
    pub obstacle: Option<PathBuf>,
}

#[derive(Deserialize, Debug, Clone)]
pub struct LinksNames {
    pub base: Vec<String>,
    pub ee: Vec<String>,
    pub used_joints: Vec<String>,
}

#[derive(Deserialize, Debug, Clone)]
struct ConfigParse{
    urdf_paths: UrdfPath,
    /// List of package name used in urdf files
    packages: Option<Vec<String>>,
    links: LinksNames,
    // starting joint values. Default = 0.0 for all
    starting_joint_values: Option<Vec<f64>>,
    // additional distance for pre grasp motion. Default = 0.03
    approach_dist: Option<f64>,
    objectives: Option<ObjectivesConfigParse>
}

#[derive(Deserialize, Debug, Clone)]
struct DefaultConfigParse{
    /// List of package name used in urdf files
    // starting joint values. Default = 0.0 for all
    // additional distance for pre grasp motion. Default = 0.03
    approach_dist: f64,
    objectives: ObjectivesConfig
}

#[derive(Clone, Debug)]
pub struct Config{
    pub urdf_paths: UrdfPath,
    pub package_paths: HashMap<String, String>,
    pub links: LinksNames,
    pub starting_joint_values: Vec<f64>,
    pub approach_dist: f64,
    pub objectives: ObjectivesConfig
}
const DEFAULT_CONF_FILE: &str = include_str!("../../configs/default.toml");
impl Config {
    pub fn from_settings_file(path_to_setting: PathBuf) -> Self {
        let mut file = File::open(path_to_setting.clone()).unwrap();
        let mut contents = String::new();
        let _res = file.read_to_string(&mut contents).unwrap();
        let res: Result<ConfigParse, toml::de::Error> = toml::from_str(&contents);
        if let Err(e) = res {
            panic!("{}", e);
        } 
        let mut conf = res.unwrap();
        let root = path_to_setting.parent().unwrap();
        conf.urdf_paths.robot = root.join(conf.urdf_paths.robot);
        if conf.urdf_paths.obstacle.is_some() {
            conf.urdf_paths.obstacle = Some(root.join(conf.urdf_paths.obstacle.unwrap()))
        }
        
        // contents = String::new();
        // let mut file = File::open(DEFAULT_CONF_FILE).expect("Default config file not found");
        // let _res = file.read_to_string(&mut contents).unwrap();
        let res: Result<DefaultConfigParse, toml::de::Error> = toml::from_str(&DEFAULT_CONF_FILE);
        if let Err(e) = res {
            panic!("{}", e);
        } 
        let default = res.unwrap();

        // Defaults : 
        let starting_joints_values = conf.starting_joint_values.unwrap_or(vec![0.0f64; conf.links.used_joints.len()]);
        let mut package_paths: HashMap<String, String> = HashMap::new();
        if let Some(packages) = conf.packages {
            for package in packages {
                let package_path = urdf_rs::utils::rospack_find(package.as_str()).unwrap();
                package_paths.insert(package, package_path);
            }
        }
        let approach_dist = conf.approach_dist.unwrap_or(default.approach_dist);
        
        //objectives 
        let mut objectives = default.objectives;
        if let Some(new_objectives) = conf.objectives {
            macro_rules! or_default {
                ($name:ident) => {{
                    if let Some(a) =  new_objectives.$name {
                        objectives.$name = a;
                    }
                }};
            }
            // TODO add macro to auto iterate
            or_default!(x_pos);
            or_default!(y_pos);
            or_default!(z_pos);
            or_default!(horizontal_grip);
            or_default!(horizontal_arm);
            or_default!(joint_limits);
            or_default!(minimize_velocity);
            or_default!(minimize_acceleration);
            or_default!(minimize_jerk);
            or_default!(maximize_manipulability);
            or_default!(self_collision);
        }
        
        Self{
            urdf_paths:conf.urdf_paths,
            package_paths: package_paths,
            links: conf.links,
            starting_joint_values: starting_joints_values,
            approach_dist: approach_dist,
            objectives: objectives
        }
    }
}
