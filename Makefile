.ONESHELL:

init_env: 
	python3 -m venv .venv
	. .venv/bin/activate
	pip install -U pip maturin

build_python :
	maturin develop -F python_wrap
