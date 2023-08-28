# ROS 2 "Humble" Installation Guide

This guide provides step-by-step instructions for installing ROS 2 "Humble" on [your operating system here, e.g., Ubuntu 22.04].


## Prerequisites

Before installing ROS 2 "Humble," ensure that you have the following prerequisites:

- A compatible operating system (e.g., Ubuntu 22.04)
- A reliable internet connection

## Installation Steps

   Open a terminal and execute the following commands to add the ROS 2 repository and install necessary dependencies:

   ```bash
   # First step：setlocale
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```
    ```bash
    #Second Step: Add ROS 2 apt repository 
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```
    ```bash
    sudo apt update && sudo apt install curl gnupg lsb-release -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    ```bash
    sudo apt update
    sudo apt upgrade
    ```
    ```bash    
    #Third Step: Install ros-humble-desktop， 
    sudo apt install ros-humble-desktop
    ```
    ```bash
    #Fourth Step: Config environment
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```
    ```bash
    #Fifth Step: Check Installation
    printenv | grep -i ROS
    ```

