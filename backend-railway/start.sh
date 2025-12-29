#!/bin/bash
set -e

echo "Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

echo "Starting FastAPI app..."
exec uvicorn index:app --host 0.0.0.0 --port $PORT
