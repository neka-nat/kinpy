setup:
	uv sync

test:
	find kinpy/. -maxdepth 1 -type f -name "*.py" | xargs uv run flake8
	uv run mypy kinpy/*.py
	uv run python -m unittest discover
