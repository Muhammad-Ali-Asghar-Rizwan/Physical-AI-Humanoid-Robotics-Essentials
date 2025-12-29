#!/bin/bash
set -e

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Start the app
uvicorn index:app --host 0.0.0.0 --port $PORT
