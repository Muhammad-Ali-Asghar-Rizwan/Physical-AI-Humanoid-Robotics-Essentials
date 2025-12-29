#!/usr/bin/env bash
set -e

# Find a usable Python interpreter
if command -v python >/dev/null 2>&1; then
	PY=python
elif command -v python3 >/dev/null 2>&1; then
	PY=python3
else
	echo "ERROR: no python interpreter found (tried 'python' and 'python3')."
	exit 1
fi

echo "Using interpreter: $PY"

if [ "$1" = "--install-only" ]; then
	echo "Installing Python dependencies (install-only)..."
	"$PY" -m pip install --upgrade pip
	"$PY" -m pip install -r requirements.txt
	echo "Dependencies installed. Exiting (install-only)."
	exit 0
fi

echo "Installing Python dependencies..."
"$PY" -m pip install --upgrade pip
"$PY" -m pip install -r requirements.txt

echo "Starting FastAPI app..."
exec "$PY" -m uvicorn index:app --host 0.0.0.0 --port $PORT
