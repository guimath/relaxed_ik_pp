.ONESHELL:

init_env: 
	python3 -m venv .venv
	. .venv/bin/activate
	pip install -U pip maturin

build_python :
	maturin develop -F python_wrap

vis_xarm: 	
	cargo run --release --example reach -- -s configs/xarm6.toml  -t 10 10 10 -c 0.785 -0.785 1.3

vis_ur5: 	
	cargo run --release --example reach -- -s configs/ur5.toml  -t 10 10 10 -c 0.785 -0.785 1.7

debug: 
	RUST_LOG=none,relaxed_ik_lib=debug 
	
ik_scan_named:
	cargo run --release --example metric -- -s configs/xarm6.toml -z 0.3 -d pose1_ik.bin --sample-per-axis 400

motion_scan_xarm:
	cargo run --release --example metric -- -s configs/xarm6.toml -m motion -z 0.3 -d pose1_motion_dist.bin --sample-per-axis 400

plot_metric:
	cargo run --release --example plot_metric -- -d ex_out/xarm6/data/pose1_ik.bin

plot_all_metric : 
	find ex_out/xarm6/data/*.bin -name '*.bin' | xargs -I {} cargo run --release --example plot_metric -- -d {}