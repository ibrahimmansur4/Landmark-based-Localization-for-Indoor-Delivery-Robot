# Start from the base image provided
FROM ibrahimmansur4/my_fyp:v2.1

# Update package lists and install required packages
RUN apt-get update && \
    apt-get -y install \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        xauth \
        x11-apps \
        git && \
    rm -rf /var/lib/apt/lists/*

# Set up environment variables
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/root/.Xauthority

# Install ROS dependencies
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Set the working directory
WORKDIR /root/catkin_ws

# Copy the source code from the host
COPY . .

# Run any additional commands or entrypoint scripts here
# ...

# Command to run when starting the container
CMD ["/bin/bash"]