FROM python:3.11-slim

WORKDIR /app

# Copy all files first
COPY . /app/

# Install dependencies
RUN python -m pip install --upgrade pip && python -m pip install -r requirements.txt

EXPOSE 8000

CMD ["uvicorn", "index:app", "--host", "0.0.0.0", "--port", "8000"]