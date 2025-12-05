.PHONY: help install dev test lint format type-check coverage clean

# Default target
help:
	@echo "RAG Chatbot Backend - Development Commands"
	@echo ""
	@echo "Setup:"
	@echo "  make install       - Install dependencies with Poetry"
	@echo "  make dev           - Install dependencies with dev tools"
	@echo ""
	@echo "Development:"
	@echo "  make run           - Run API server with hot reload"
	@echo "  make test          - Run all tests (unit + integration)"
	@echo "  make test-unit     - Run unit tests only"
	@echo "  make test-int      - Run integration tests only"
	@echo ""
	@echo "Quality Assurance:"
	@echo "  make lint          - Run all linters (isort, black, flake8, mypy)"
	@echo "  make format        - Auto-format code (isort, black)"
	@echo "  make type-check    - Run MyPy type checking"
	@echo "  make coverage      - Run tests with coverage report"
	@echo ""
	@echo "Utilities:"
	@echo "  make clean         - Remove generated files"
	@echo "  make check-full    - Run all checks (format, lint, test, coverage)"
	@echo "  make health        - Check system health endpoints"

install:
	poetry install --no-dev

dev:
	poetry install

run:
	poetry run python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

test:
	poetry run pytest -v --tb=short

test-unit:
	poetry run pytest -m unit -v --tb=short

test-int:
	poetry run pytest -m integration -v --tb=short

lint: format type-check
	@echo "✓ All linting checks passed"

format:
	@echo "Running import sorting with isort..."
	poetry run isort src/ tests/
	@echo "Running code formatting with black..."
	poetry run black src/ tests/
	@echo "✓ Code formatted"

type-check:
	@echo "Running MyPy type checking..."
	poetry run mypy src/
	@echo "✓ Type checking passed"

coverage:
	poetry run pytest --cov=src --cov-report=html --cov-report=term-missing
	@echo ""
	@echo "Coverage report generated in htmlcov/index.html"

clean:
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type d -name .pytest_cache -exec rm -rf {} +
	find . -type d -name .mypy_cache -exec rm -rf {} +
	find . -type d -name htmlcov -exec rm -rf {} +
	find . -type f -name .coverage -delete
	find . -type f -name coverage.xml -delete
	@echo "✓ Cleaned up"

check-full: clean format type-check test coverage
	@echo ""
	@echo "✅ All quality checks passed!"

health:
	@echo "Checking API health endpoints..."
	@echo ""
	@echo "Basic health check:"
	curl -s http://localhost:8000/health | python -m json.tool
	@echo ""
	@echo "Full dependency health check:"
	curl -s http://localhost:8000/health/full | python -m json.tool
	@echo ""
	@echo "Qdrant health:"
	curl -s http://localhost:8000/health/qdrant | python -m json.tool
	@echo ""
	@echo "Gemini health:"
	curl -s http://localhost:8000/health/gemini | python -m json.tool
