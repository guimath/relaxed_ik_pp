# Relaxed IK pp

Fork of relaxed IK core by University of Wisconsin-Madison's Graphics Group
part of Master thesis in robotics at NUS, School of Computing.

Submodule of GenMoP

## Introduction
The idea behind this is to formulate a grasp pose to do a high level motion planning: by only providing shape and position, the robot should be able to find a good grasp position, depending on its current position and the environment. 

## Getting Started 

1. [Install Rust](https://www.rust-lang.org/learn/get-started)
2. Compile:
    ```bash
    cargo build
    ```
   The compiled library is at `/target/debug/librelaxed_ik_lib.so`
3. Run a small demo:
    ```bash
    cargo run --bin relaxed_ik_bin
    ```

To compile the python wrapper use maturin:
1. start by initializing the venv
```bash
	python3 -m venv .venv
	. .venv/bin/activate
	pip install -U pip maturin
```
2. build using
```bash
	maturin develop
```

Both can be done by running `make maturin` (still need to activate venv to directly run maturin commands)

### Use your own robot
1. Place your robot's URDF under `configs/urdfs/`
2. Make a setting file. Examples are under `configs/example_settings`
