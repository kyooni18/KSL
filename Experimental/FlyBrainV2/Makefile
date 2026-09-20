.PHONY: train validate clean

PYTHON ?= python3

train:
	$(PYTHON) train.py

validate:
	$(PYTHON) validate.py

clean:
	rm -f brain_v2.safetensors training_report.json
	rm -rf __pycache__
