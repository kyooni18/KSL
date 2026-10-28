# KSL developer entry points. See README.md.
PYTHON ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)

.PHONY: all backend sim ascent setup test test-c test-py qualification clean help

all: backend sim ascent                ## build native components

ascent:                                ## build Rust ascent guidance for the Python interpreter
	@if [ "$$(uname -s)" = Darwin ] && [ "$$($(PYTHON) -c 'import platform; print(platform.machine())')" = x86_64 ]; then \
		cargo build --release --target x86_64-apple-darwin --manifest-path rust/ascent/Cargo.toml; \
	else cargo build --release --manifest-path rust/ascent/Cargo.toml; fi

backend:                               ## build CLanding/build/landing_backend
	$(MAKE) -C CLanding

sim:                                   ## build ShuttleSim/build/shuttlesim
	cmake -S ShuttleSim -B ShuttleSim/build >/dev/null
	cmake --build ShuttleSim/build -j >/dev/null

setup:                                 ## create .venv with runtime + test dependencies
	python3 -m venv .venv
	.venv/bin/pip install --disable-pip-version-check -q -r requirements.txt

test: test-c test-py                   ## fast offline regression gate (C + Python)

test-c: backend
	$(MAKE) -C CLanding test

test-py: backend sim ascent
	$(PYTHON) -m pytest -q tests Validation

qualification: backend                 ## slow physical gates (~4 min; includes a known open gate)
	$(MAKE) -C CLanding qualification

clean:
	$(MAKE) -C CLanding clean
	rm -rf ShuttleSim/build
	cargo clean --manifest-path rust/ascent/Cargo.toml

help:
	@grep -E '^[a-z-]+:.*##' $(MAKEFILE_LIST) | sed 's/:.*##/ —/'
