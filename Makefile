CARGO_EXAMPLE := cargo run --release --example
ROBOT := xarm6
CONFIG_FILE := configs/$(ROBOT).toml
LOG_LEVEL := warn  # debug, info, warn, none 
ZOOM_DIST := 1.3
NAME = place_holder.bin
SAMPLES = 50
ARGS := 
EXAMPLE:= reach
DATA_FOLDER := ex_out/$(ROBOT)/data
debug := RUST_LOG=none,relaxed_ik_lib=$(LOG_LEVEL)


.ONESHELL:

init_env: 
	python3 -m venv .venv
	. .venv/bin/activate
	pip install -U pip maturin

build_python :
	maturin develop -F python_wrap

reach_vis_no_bottle:
	$(debug) $(CARGO_EXAMPLE) reach -- -s $(CONFIG_FILE) -t 10 10 10 -c 0.785 -0.785 $(ZOOM_DIST)

reach_vis: 
	$(debug) $(CARGO_EXAMPLE) reach -- -s $(CONFIG_FILE) -m visual-only $(ARGS)

reach_full: 
	$(debug) $(CARGO_EXAMPLE) reach -- -s $(CONFIG_FILE) -m full $(ARGS)
	 
	
metric_ik:
	$(debug) $(CARGO_EXAMPLE) metric -- -s $(CONFIG_FILE) $(ARGS)

metric_motion:
	$(debug) $(CARGO_EXAMPLE) metric -- -s $(CONFIG_FILE) -m motion $(ARGS)

plot_metric:
	$(CARGO_EXAMPLE) plot_metric -- $(ARGS)

plot_all_metric : 
	find $(DATA_FOLDER)/*.bin -name '*.bin' | xargs -I {} $(CARGO_EXAMPLE) plot_metric -- -d {}

metric_and_plot : 
	$(debug) $(CARGO_EXAMPLE) metric -- -s $(CONFIG_FILE) -d $(NAME) --sample-per-axis $(SAMPLES) $(ARGS)
	$(CARGO_EXAMPLE) plot_metric -- -d $(DATA_FOLDER)/$(NAME)

help:
	$(CARGO_EXAMPLE) $(EXAMPLE) -- -h
	