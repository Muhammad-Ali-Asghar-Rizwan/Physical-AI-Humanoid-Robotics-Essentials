#!/usr/bin/env bash
set -e

# If there's a Dockerfile at repository root, let Railway use it and exit.
if [ -f "Dockerfile" ]; then
  echo "Dockerfile present; letting Railway build the Docker image. Exiting start.sh." 
  exit 0
fi

# Fallback: start the Python FastAPI app from the backend folder
if [ -d "my_backend_project/backend" ]; then
  cd my_backend_project/backend
elif [ -d "backend" ]; then
  cd backend
fi

if [ -f "requirements.txt" ]; then
  pip install --no-cache-dir -r requirements.txt
fi

exec uvicorn main:app --host 0.0.0.0 --port ${PORT:-8000}
#!/bin/bash
pip install -r requirements.txt
python -m uvicorn main:app --host=0.0.0.0 --port=$PORT