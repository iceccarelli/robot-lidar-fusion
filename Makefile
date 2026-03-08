.PHONY: install dev test lint format clean docker run

install:
	pip install -e .

dev:
	pip install -e ".[dev]"

test:
	pytest -v

lint:
	ruff check .

format:
	ruff check --fix .

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	rm -rf dist build .pytest_cache htmlcov .coverage .ruff_cache

docker:
	docker compose build

run:
	python scripts/run_robot.py --cycles 100
