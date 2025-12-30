FROM python:3.11-slim

# Set working directory
WORKDIR /app
# Ensure requirements are present and install only them first (layer cache)
COPY requirements.txt ./requirements.txt

# Prevent Python from writing pyc files and buffer stdout/stderr
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
# Install dependencies (install only requirements to leverage cache)
RUN python -m pip install --upgrade pip \
    && python -m pip install -r requirements.txt

# Copy application code (current build context is backend-railway folder)
COPY . /app/

# Expose port
EXPOSE 8000

# Start command
CMD ["uvicorn", "index:app", "--host", "0.0.0.0", "--port", "8000"]
