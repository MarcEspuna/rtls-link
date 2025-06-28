# rtls-link
Real-time positioning system based on UWB technology

[![Build and Test ESP32 Applications](https://github.com/marcespuna/rtls-link/actions/workflows/build-test-docker.yml/badge.svg)](https://github.com/marcespuna/rtls-link/actions/workflows/build.yml)

## Development Environment

This project includes a containerized PlatformIO development environment for easy setup and consistent builds.

### Using Docker

Pull the pre-built development environment:
```bash
docker pull ghcr.io/YOUR_GITHUB_USERNAME/rtls-link-platformio:latest
```

Run the development environment:
```bash
cd docker
docker-compose up -d
docker exec -it platformio-development bash
```

### Building the Docker Image Locally

If you prefer to build the image locally:
```bash
cd docker
docker build -t platformio-env:latest .
docker-compose up -d
```

## Continuous Integration

This project includes automated CI/CD workflows that test compilation of both ESP32 environments:

### Build Workflow
The CI uses the containerized PlatformIO environment for faster, more consistent builds that match your local development setup exactly.

### Features
- ✅ **Parallel builds** for both ESP32 environments
- ✅ **Intelligent caching** of PlatformIO dependencies
- ✅ **Build artifact upload** (firmware binaries, ELF files)
- ✅ **Memory usage analysis** and build size reporting
- ✅ **Automatic triggering** on push/PR to main/develop branches

The CI automatically runs on:
- Push to `main` or `develop` branches
- Pull requests targeting `main` or `develop`
- Manual workflow dispatch
