use crate::spacetime::robot::Robot;
use nalgebra::{UnitQuaternion, Vector3, Vector6};

use crate::utils::config_parser::Config;
use serde::{Deserialize, Serialize};
use std::path::Path;
#[derive(Serialize, Deserialize)]
pub struct VarsConstructorData {
    // pub urdf: String,
    pub link_radius: f64,
    pub base_links: Vec<String>,
    pub ee_links: Vec<String>,
    starting_config: Vec<f64>,
}

#[derive(Clone)]
pub struct RelaxedIKVars {
    pub robot: Robot,
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>,
    pub goal_positions: Vec<Vector3<f64>>,
    pub goal_quats: Vec<UnitQuaternion<f64>>,
    pub tolerances: Vec<Vector6<f64>>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>,
}
impl RelaxedIKVars {
    pub fn from_local_settings<P: AsRef<Path> + Clone>(path_to_setting: P) -> Self {
        let config: Config = Config::from_settings_file(path_to_setting);
        Self::from_config(config)
    }

    pub fn from_config(config: Config) -> Self {
        let _chain = k::Chain::<f64>::from_urdf_file(config.urdf_paths.robot.clone()).unwrap();
        let num_chains = config.links.base.len();

        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for _ in 0..num_chains {
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let urdf = &std::fs::read_to_string(config.urdf_paths.robot).unwrap();
        let robot = Robot::from_urdf(urdf, &config.links.base, &config.links.ee);

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let poses = robot.get_ee_pos_and_quat_immutable(&config.starting_joint_values);
        assert!(poses.len() == num_chains);
        for pose in poses {
            init_ee_positions.push(pose.0);
            init_ee_quats.push(pose.1);
        }

        RelaxedIKVars {
            robot,
            init_state: config.starting_joint_values.clone(),
            xopt: config.starting_joint_values.clone(),
            prev_state: config.starting_joint_values.clone(),
            prev_state2: config.starting_joint_values.clone(),
            prev_state3: config.starting_joint_values.clone(),
            goal_positions: init_ee_positions.clone(),
            goal_quats: init_ee_quats.clone(),
            tolerances,
            init_ee_positions,
            init_ee_quats,
        }
    }

    // for webassembly
    pub fn from_jsvalue(configs: VarsConstructorData, urdf: &str) -> Self {
        let num_chains = configs.base_links.len();

        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for _ in 0..num_chains {
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let robot = Robot::from_urdf(urdf, &configs.base_links, &configs.ee_links);

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let poses = robot.get_ee_pos_and_quat_immutable(&configs.starting_config);
        assert!(poses.len() == num_chains);

        for pose in poses {
            init_ee_positions.push(pose.0);
            init_ee_quats.push(pose.1);
        }

        RelaxedIKVars {
            robot,
            init_state: configs.starting_config.clone(),
            xopt: configs.starting_config.clone(),
            prev_state: configs.starting_config.clone(),
            prev_state2: configs.starting_config.clone(),
            prev_state3: configs.starting_config.clone(),
            goal_positions: init_ee_positions.clone(),
            goal_quats: init_ee_quats.clone(),
            tolerances,
            init_ee_positions,
            init_ee_quats,
        }
    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }

    pub fn reset(&mut self, init_state: Vec<f64>) {
        self.prev_state3 = init_state.clone();
        self.prev_state2 = init_state.clone();
        self.prev_state = init_state.clone();
        self.xopt = init_state.clone();
        self.init_state = init_state.clone();

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let poses = self.robot.get_ee_pos_and_quat_immutable(&init_state);

        for pose in poses {
            init_ee_positions.push(pose.0);
            init_ee_quats.push(pose.1);
        }

        self.init_ee_positions = init_ee_positions.clone();
        self.init_ee_quats = init_ee_quats.clone();
    }
}
