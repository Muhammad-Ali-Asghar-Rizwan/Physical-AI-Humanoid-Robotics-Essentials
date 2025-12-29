#!/usr/bin/env bash
set -e

echo "Installing Python dependencies..."
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo "Starting FastAPI app..."
exec python -m uvicorn index:app --host 0.0.0.0 --port $PORT
