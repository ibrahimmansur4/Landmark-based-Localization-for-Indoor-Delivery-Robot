# ROS Noetic (desktop-full) Installation Guide

Follow the steps below to install ROS Noetic (desktop-full) on your system.

## Configure Ubuntu Repositories

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse" by following the Ubuntu guide.

## Setup your sources.list

Run the following command:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Set up your keys

```bash
sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## Installation

First, make sure your Debian package index is up-to-date:

```bash
sudo apt update
```

### Desktop-Full Install

```bash 
sudo apt install ros-noetic-desktop-full
```

## Environment Setup

Source the setup script in every bash terminal you use ROS in:

```bash
source /opt/ros/noetic/setup.bash
```

OR (Recommended):
- You can automatically source this script every time a new shell is launched. Run the following commands:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Dependencies for Building Packages
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Initialize rosdep
    
```bash
sudo apt install python3-rosdep

sudo rosdep init

rosdep update
```

Follow these steps to install ROS Noetic (desktop-full) on your system. For more information and troubleshooting, refer to the official ROS [documentation]((http://wiki.ros.org/noetic/Installation/Ubuntu)).

## License
This project is open-source and released under the [MIT License](/LICENSE). You are free to use and modify it for your purposes.

## Acknowledgements
Special thanks to the ROS community.