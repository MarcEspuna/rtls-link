# Use Ubuntu 24.04 LTS as the base image
FROM ubuntu:24.04

# Set environment variables to avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install essential packages
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    curl \
    wget \
    git \
    build-essential \
    gcc \
    g++ \
    make \
    cmake \
    pkg-config \
    libssl-dev \
    libffi-dev \
    udev \
    doxygen \
    graphviz \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for python (PlatformIO expects 'python' command)
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install PlatformIO CLI (using system pip with --break-system-packages for Docker environment)
RUN pip install platformio --break-system-packages

# Add PlatformIO to PATH (should already be there, but making sure)
ENV PATH="/root/.local/bin:${PATH}"

# Create a directory for PlatformIO projects
RUN mkdir -p /workspace/projects

# Set PlatformIO home directory
ENV PLATFORMIO_HOME="/workspace/.platformio"

# Initialize PlatformIO (downloads platforms and frameworks)
RUN pio system info

# Set working directory
WORKDIR /workspace

# Verify installation
RUN pio --version && pio system info

# Default command
CMD ["/bin/bash"] 