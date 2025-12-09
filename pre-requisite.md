# System Requirements & Prerequisites

## Hardware

| Component | Minimum | Recommended | Check |
|-----------|---------|-------------|-------|
| **GPU** | NVIDIA Ampere+ (≥6 GB VRAM) | RTX 3060+ | `nvidia-smi` |
| **RAM** | 16 GB | 32 GB | `free -h` |
| **Storage** | 50 GB free | 150+ GB free | `df -h .` |
| **CPU** | Modern multicore | 8+ cores | `lscpu` |

## Operating System

| Requirement | Version | Check |
|-------------|---------|-------|
| **Ubuntu** | 22.04 LTS | `lsb_release -a` |
| **Kernel** | Recent (5.15+) | `uname -r` |
| **User** | sudo + docker group | `groups` |

## NVIDIA Software Stack

| Component | Required | Check |
|-----------|----------|-------|
| **NVIDIA Driver** | 580+ (CUDA 13 compatible) | `nvidia-smi` |
| **CUDA Toolkit** | 12.x or 13.x | `nvcc --version` |
| **TensorRT** | Latest compatible | `dpkg -l \| grep nvinfer` |
| **Container Toolkit** | nvidia-container-toolkit | `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi` |


