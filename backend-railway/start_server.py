#!/usr/bin/env python3
import subprocess
import sys
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Look for requirements.txt in the script directory
requirements_path = os.path.join(script_dir, 'requirements.txt')

if os.path.exists(requirements_path):
    print("Installing requirements from requirements.txt...")
    try:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-r', requirements_path])
        print("Requirements installed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Error installing requirements: {e}")
        sys.exit(1)
else:
    print(f"requirements.txt not found at {requirements_path}")
    print("Available files:", os.listdir(script_dir))

# Start the application
try:
    from backend.main import app
    print("Successfully imported app from backend.main")
except ImportError as e:
    print(f"Error importing app: {e}")
    sys.exit(1)

if __name__ == "__main__":
    try:
        import uvicorn
        port = int(os.environ.get("PORT", 8000))
        print(f"Starting server on port {port}")
        uvicorn.run(app, host="0.0.0.0", port=port)
    except ImportError:
        print("uvicorn not found, installing it now...")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'uvicorn[standard]'])
        import uvicorn
        port = int(os.environ.get("PORT", 8000))
        print(f"Starting server on port {port}")
        uvicorn.run(app, host="0.0.0.0", port=port)