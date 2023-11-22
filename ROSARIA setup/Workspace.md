# ROSARIA Setup

This repository contains the setup instructions for your ROS project. Follow the steps below to set up the project on your system.

## Prerequisites

Make sure you have the following installed on your system:

- ROS Noetic
- `rosdep`
- `make`
- `g++`

## ROS Workspace Setup

1. Create a catkin workspace for ROS:

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   catkin_init_workspace
   cd ~/catkin_ws
   catkin_make
   ```
2. Clone the required repositories:
    
   ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/amor-ros-pkg/rosaria.git
    git clone https://github.com/reedhedges/AriaCoda.git
    git clone https://github.com/NKU-MobFly-Robotics/p3dx.git

    ```

3. Install ROS dependencies:
    
   ```bash
   cd 
   rosdep install rosaria
   ```
4. Build the workspace:
    
   ```bash
   cd ~/catkin_ws
   catkin_make
    ```
5. Build Aria from source (don't know if required try if catkin_make gives error) otherwise jump to step 6:
    
   ```bash
    sudo apt install make g++
    cd ~/catkin_ws/src/AriaCoda
    make -j6
   ```
6. Create another ROS workspace for the project:
    
   ```bash
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/
    catkin_make
   ```
7. Source the ROS environment:
    
   ```bash
    source ~/catkin_ws/devel/setup.bash
    source ~/ros_ws/devel/setup.bash
   ```
## Usage
To use and run the ROS project, follow these steps:

1. Make sure you have the necessary hardware and connections set up.
2. Run the required launch files using the appropriate ROS commands.
3. Interact with the project using the provided user interface or scripts.

## License
This project is open-source and released under the [MIT License](/LICENSE). You are free to use and modify it for your purposes.

Feel free to contribute, report issues, or suggest improvements by creating a pull request or opening an issue.

## Acknowledgements
Special thanks to the contributors of the ROS projects used in this repository. The links to their original repositories are provided below:

- [amor-ros-pkg/rosaria](https://github.com/amor-ros-pkg/rosaria)
- [reedhedges/AriaCoda](https://github.com/reedhedges/AriaCoda)
- [NKU-MobFly-Robotics/p3dx](https://github.com/NKU-MobFly-Robotics/p3dx)

