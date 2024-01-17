# Use an official Ubuntu as a parent image
FROM ubuntu:latest

# Set environment variables to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install PlatformIO Core
RUN pip3 install -U platformio

# Set a working directory
WORKDIR /robot-dog

# Copy your project files into the container (assuming your source code is in the same directory as the Dockerfile)
COPY . /robot-dog

# Define the default command to build and upload your STM32 code
CMD ["platformio", "run", "-t", "upload"]
