# Relaxed IK pp

This project is part of a Master's thesis in robotics at the National University of Singapore, School of Computing.

Thesis: Leveraging Rust for real time low computational Robotics
Author: Guilhem Mathieux (@guimath)
Advisor: Lin Shao

This repository is a Fork of [relaxed IK core](https://github.com/uwgraphics/relaxed_ik_core.git) by University of Wisconsin-Madison's Graphics Group. It provides some optimizations, code improvements and added functionality. It also incorporates a motion planner to have complete functionality for robotics motion control.


## Introduction

The project focuses on pre-grasp motion but should be adaptable to much more. 
The base idea is to formulate any task as an optimization problem. Here, the task is to reach a grasp pose: the robot should be able to find a good grasp position and a way to get to that position depending on its current position and the environment. All of that should be done in real-time, with low computational power.

## Getting Started 

### Build 
1. [Install Rust](https://www.rust-lang.org/learn/get-started)
2. Run the interactive demo:
    ```bash
    cargo run --example reach
    ```
or ```bash
    make reach_full
    ```

To compile the python wrapper use maturin:
1. start by initializing the venv: 
```bash
    make init_env
```
or 
```bash
	python3 -m venv .venv
	. .venv/bin/activate
	pip install -U pip maturin
```
2. build using
```bash
    make build_python
```
or
```bash
	maturin develop -F python_wrap
```


### Config files
The config files are written in toml, and allow for robot specific modifications. 
- urdf_paths: Paths relative to config file
    - robot
    - obstacles
- packages: List of package names used in urdf files (not mandatory, can help speed up urdf load)
- links:
    - base: name of the base link
    - ee: name of end-effector target 
    - used_joints: list of all joints to be used
- starting_joint_values: joint values to be considered as reset. Default = 0.0 for all
- approach_dist: additional distance for pre grasp motion. Default = 0.03

For examples of config files see [xarm6 file](configs/xarm6.toml) or [ur5 file](configs/ur5.toml).

Objective weights and cost function can also be specified for example:
```toml
[objectives.x_pos]
func= {Groove={t = 0.0, d = 2, c = 0.1, f = 10.0, g = 2}}
weight= 70
```
An objective can be fully disabled by making the weight = 0.

For more examples and all the defaults objective configs, see the [default file](configs/default.toml)

## Examples

- [bench.rs](examples/bench.rs) runs a large batch of gradients calculation on the different active objectives to test time needed on each (gradient being the most computational heavy operation as it requires many objective calls for each gradient)
- [cost_viz.rs](examples/cost_viz.rs) simple visualization tool to plot a given cost function. See the [example.toml](examples/cost_configs/example.toml) file for an example of plot with 3 different cost function.
- [metric.rs](examples/metric.rs) Scan large area around the robot either computing IK or Motion and saves data to a bin file.
- [plot_metric.rs](examples/plot_metric.rs) plots a metric bin file to a heatmap.
- [reach.rs](examples/reach.rs) Visualize robot and see motion and inverse kinematic results in simulation

The metric and reach examples have some helper commands, see the [Makefile](Makefile)   
