# Backend

This directory contains the FastAPI backend for the AI-Course-Book platform.

## Setup

1. Navigate into the `backend/` directory.
2. Initialize `uv` if not already done: `uv init .`
3. Install dependencies: `uv sync`
4. Set environment variables in a `.env` file (see `.env.example`).

## Running the application

`uv run uvicorn src.main:app --host 0.0.0.0 --port 8000`
