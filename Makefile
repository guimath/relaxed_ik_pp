.ONESHELL:

maturin :
# Creating and setting python venv
	python3 -m venv .venv
	. .venv/bin/activate
	pip install -U pip maturin
# building project and wrapper
	maturin develop