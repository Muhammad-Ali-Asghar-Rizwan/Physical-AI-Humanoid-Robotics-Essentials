FROM python:3.11-slim

WORKDIR /app

# Copy requirements
COPY requirements.txt ./requirements.txt

# Prevent Python from writing pyc files and buffer stdout/stderr
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Install dependencies
RUN python -m pip install --upgrade pip \
    && python -m pip install -r requirements.txt

# Copy application code
COPY . /app/

EXPOSE 8000

CMD ["uvicorn", "index:app", "--host", "0.0.0.0", "--port", "8000"]