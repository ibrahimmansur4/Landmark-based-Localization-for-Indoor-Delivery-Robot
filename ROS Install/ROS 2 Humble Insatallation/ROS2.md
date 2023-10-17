# ROS 2 "Humble" Installation Guide

This guide provides step-by-step instructions for installing ROS 2 "Humble" on [your operating system here, e.g., Ubuntu 22.04].


## Prerequisites

Before installing ROS 2 "Humble," ensure that you have the following prerequisites:

- A compatible operating system (e.g., Ubuntu 22.04)
- A reliable internet connection

## Installation Steps

   Open a terminal and execute the following commands to add the ROS 2 repository and install necessary dependencies:

### First Step: Set Locale
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

locale  # verify settings
```
### Second Step: Add ROS 2 apt repository 
```bash
sudo apt install software-properties-common

sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade
```
### Third Step: Install ros-humble-desktop
```bash    
 
sudo apt install ros-humble-desktop
```
### Fourth Step: Config environment
```bash

source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### Fifth Step: Check Installation
```bash

printenv | grep -i ROS
```

### Gazebo Installation
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```
Follow these steps to successfully install ROS 2 "Humble" on your system.

For more information and troubleshooting, refer to the official ROS 2 [documentation](https://docs.ros.org/en/humble/Installation.html).

## License
This project is open-source and released under the [MIT License](/LICENSE). You are free to use and modify it for your purposes.

## Acknowledgements
Special thanks to the ROS community and contributors for their support and contributions to ROS 2.