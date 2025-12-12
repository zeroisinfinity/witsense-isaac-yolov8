#!/bin/bash

set -e

PYTHON_BIN="/usr/bin/python3"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Isaac Sim YOLOv8 Launcher ===${NC}\n"

echo "Checking dependencies..."

if ! command -v $PYTHON_BIN &> /dev/null; then
    echo -e "${RED}❌ Python3 not found${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Python3: $PYTHON_BIN${NC}"

if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}❌ ROS2 not found. Installing...${NC}"
    sudo apt update
    sudo apt install -y ros-jazzy-ros-base
    source /opt/ros/jazzy/setup.bash
fi
echo -e "${GREEN}✅ ROS2${NC}"

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null

export ROS_DOMAIN_ID=0

MISSING=""
$PYTHON_BIN -c "import rclpy" 2>/dev/null || MISSING="$MISSING python3-rclpy"
$PYTHON_BIN -c "import cv_bridge" 2>/dev/null || MISSING="$MISSING ros-jazzy-cv-bridge"
$PYTHON_BIN -c "import cv2" 2>/dev/null || MISSING="$MISSING opencv-python"
$PYTHON_BIN -c "import ultralytics" 2>/dev/null || MISSING="$MISSING ultralytics"
$PYTHON_BIN -c "import numpy" 2>/dev/null || MISSING="$MISSING numpy"

if [ -n "$MISSING" ]; then
    echo -e "${YELLOW}Installing missing packages...${NC}"
    sudo apt install -y python3-rclpy ros-jazzy-cv-bridge 2>/dev/null || true
    $PYTHON_BIN -m pip install opencv-python ultralytics numpy --break-system-packages
fi
echo -e "${GREEN}✅ All dependencies installed${NC}\n"

if [ ! -d "src" ]; then
    echo -e "${RED}❌ src/ directory not found. Run from repo root.${NC}"
    exit 1
fi

if [ ! -f "src/camera.py" ]; then
    echo -e "${RED}❌ src/camera.py not found${NC}"
    exit 1
fi

echo -e "${GREEN}Checking Isaac Sim camera feed...${NC}"

source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0

if ros2 topic list 2>/dev/null | grep -q "/sim/camera/rgb"; then
    echo -e "${GREEN}✅ Camera topic found: /sim/camera/rgb${NC}\n"
else
    echo -e "${RED}❌ Camera topic not found!${NC}"
    echo -e "${YELLOW}Please:${NC}"
    echo -e "  1. Open Isaac Sim"
    echo -e "  2. Load simple_scene.usd"
    echo -e "  3. Press PLAY button"
    echo -e "  4. Wait for simulation to start\n"
    echo -e "${YELLOW}Press 'y' when ready to continue...${NC}"
    
    while true; do
        read -n 1 -r key
        if [[ $key == "y" ]] || [[ $key == "Y" ]]; then
            echo -e "\n${GREEN}Checking again...${NC}"
            if ros2 topic list 2>/dev/null | grep -q "/sim/camera/rgb"; then
                echo -e "${GREEN}✅ Camera topic found!${NC}\n"
                break
            else
                echo -e "${RED}❌ Still not found. Check Isaac Sim and try again.${NC}"
                echo -e "${YELLOW}Press 'y' to retry or Ctrl+C to exit...${NC}"
            fi
        fi
    done
fi

echo -e "${GREEN}Starting detection node...${NC}\n"

cd src
$PYTHON_BIN camera.py
