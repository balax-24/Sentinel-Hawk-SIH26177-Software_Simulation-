# Multi-architecture Dockerfile for SIH Search & Rescue 3D Reconstruction
# Supports x86_64 and ARM64 (Raspberry Pi 4 / Edge SBCs)

FROM python:3.11-slim

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV HEADLESS=1

# Install system dependencies required for OpenCV, Open3D, and ONNX Runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    wget \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Install Python requirements
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Pre-create models and data directories
RUN mkdir -p models data/input data/output

# Pre-fetch the lightweight MiDaS ONNX model for offline/air-gapped field deployment
RUN wget -q -O models/model-small.onnx \
    https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx || true

# Copy project source code and configuration
COPY config/ ./config/
COPY app/ ./app/
COPY data/ ./data/

# Default entrypoint
ENTRYPOINT ["python", "-m", "app.main"]
CMD ["--input", "data/input/image.jpg", "--headless"]
