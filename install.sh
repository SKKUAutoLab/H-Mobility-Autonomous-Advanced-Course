#!/bin/bash

# Update package lists
sudo apt-get update

# Install python3-pip using apt
sudo apt-get install -y python3-pip 

# Install Python packages using pip
pip install ultralytics setuptools==58.2.0 opencv-python pyserial 
