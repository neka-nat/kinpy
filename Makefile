setup:
	poetry install --no-interaction
	poetry run pip install -e .

test:
	find kinpy/. -maxdepth 1 -type f -name "*.py" | xargs poetry run flake8
	mypy kinpy/*.py
	poetry run python -m unittest discover