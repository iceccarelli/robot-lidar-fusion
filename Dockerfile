FROM python:3.11-slim AS base

LABEL maintainer="iceccarelli"
LABEL description="Robot LiDAR Fusion — autonomous robot control stack with sensor fusion"

WORKDIR /app

# Install system dependencies for numpy and OpenCV
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        libgl1 \
        libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

# Copy dependency specification first for layer caching
COPY pyproject.toml README.md ./

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir .

# Copy the full project
COPY . .

# Install in editable mode
RUN pip install --no-cache-dir -e .

# Default: run the control loop
CMD ["python", "scripts/run_robot.py", "--cycles", "100"]
