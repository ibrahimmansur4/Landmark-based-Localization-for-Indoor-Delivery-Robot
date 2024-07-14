# FYP Setup Guide using Docker

This guide provides step-by-step instructions for setting up your Final Year Project (FYP) using Docker containers.

## Table of Contents
- [Environment Setup for FYP Docker](#environment-setup-for-fyp-docker)
- [Docker Installation - OSRF](#docker-installation---osrf)
- [Basic Container Creation](#basic-container-creation)
- [Container Creation with GUI Access](#container-creation-with-gui-access)
- [Install Git in the Container](#install-git-in-the-container)
- [Setup ROSARIA](#setup-rosaria)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## Environment Setup for FYP Docker

First, set up the necessary environment for your  Docker:
- Go [here](/Docker%20Related/Installation%20Docker.md) for the docker installation guide.
## Docker Installation - OSRF
To install OSRF in Docker, use the following command:
```bash
docker pull osrf/ros:noetic-desktop-full
```
## Basic Container Creation
To create a basic container without GUI access, run the following commands:
```bash
docker run --name ros1_fyp -it osrf/ros:noetic-desktop-full

apt-get update
```
## Container Creation with GUI Access
- See [this link](http://wiki.ros.org/action/login/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2) for Linux and Intel Graphics solutions.
```bash
docker run --net=host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --device=/dev/dri:/dev/dri --group-add video --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --name=ros_ws_fyp -it osrf/ros:noetic-desktop-full bash

apt-get update

apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && rm -rf /var/lib/apt/lists/

# GUI setup for Docker
apt-get update && apt-get install -y xauth x11-apps

# Grant Docker access in the main OS terminal outside the Docker container
sudo xhost +local:docker

# Start the container
docker start <container>

# Run a premade/stopped container
docker exec -it <container> bash

# Check the version of the system
lsb_release -a
```

## Install Git in the Container
```bash
apt-get update

apt-get install git

git config --global user.name "Your Name"
```

## Setup ROSARIA
Follow the steps below to setup ROSARIA in your Docker container:
- Go [here](/ROSARIA%20setup/Workspace.md) to the ROSARIA setup guide.

## Acknowledgements
Special thanks to the OSRF and contributors for their support and contributions to the Docker project.

## License
This guide is open-source and released under the [MIT License](/LICENSE). You are free to use and modify it for your purposes.
