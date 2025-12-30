#!/usr/bin/env python3
import subprocess
import sys
import os

# Install requirements if not already installed
if os.path.exists('requirements.txt'):
    print("Installing requirements from requirements.txt...")
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-r', 'requirements.txt'])
else:
    print("No requirements.txt found!")

# Start the application
from backend.main import app

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)