# Docker Installation

This guide provides step-by-step instructions for installing Docker on Ubuntu 22.04 Linux.

## Table of Contents
- [First Time Installation](#first-time-installation)
- [Other Installation Methods](#other-installation-methods)
- [Enable Boot on Docker](#enable-boot-on-docker)
- [Removing Sudo](#allowing-docker-access-without-sudo)
- [Removing Docker](#removing-docker)
- [Additional Resources](#additional-resources)
- [Basic Docker Commands](#basic-docker-commands)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## First Time Installation

```bash
# Add Docker's official GPG key:
sudo apt-get update

sudo apt-get install ca-certificates curl gnupg

sudo install -m 0755 -d /etc/apt/keyrings

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

# Install Docker packages:
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Docker images error fix:
systemctl start docker
```

## Other Installation Method

```bash
# Alternative installation method using snap:
sudo apt install docker.io

sudo snap install docker

docker --version

```

## Enable Boot on Docker
```bash
# Check if Docker is enabled at boot:
systemctl is-enabled docker

# If not enabled, enable it:
sudo systemctl enable docker.service

sudo systemctl enable containerd.service
```


## Allowing Docker Access without sudo

```bash
sudo chmod 666 /var/run/docker.sock  # Provides access without requiring sudo

sudo groupadd docker

sudo usermod -aG docker $USER

newgrp docker
```

## Removing Docker
```bash
# Uninstall Docker and related packages:
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done
sudo apt-get purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin docker-ce-rootless-extras
sudo rm -rf /var/lib/docker
sudo rm -rf /var/lib/containerd
```


## Additional Resources
- [Docker Step By Step](https://www.simplilearn.com/tutorials/docker-tutorial/how-to-install-docker-on-ubuntu)
- [Docker Official Guide](https://docs.docker.com/engine/install/ubuntu/)


## Basic Docker Commands

Use the following basic Docker commands to manage Docker containers and images:

### Run the "hello-world" container to verify that Docker is correctly installed:

```bash
sudo docker run hello-world  # Verifies successful Docker installation

sudo docker images  # Lists all available Docker images

sudo docker ps -a  # Lists all Docker containers, both running and stopped

sudo docker ps  # Lists only the running Docker containers

sudo docker rm <container_id>  # Removes a Docker container by its container ID

sudo docker rmi <image_id>  # Removes a Docker image by its image ID

docker stop <container_id>  # Stops a running Docker container by its container ID

docker start <container_id>  # Starts a stopped Docker container by its container ID

docker exec -it <container_id> bash  # Opens a bash terminal inside a running Docker container

docker run -it <image_name> bash  # Opens a bash terminal inside a Docker container

docker run -d <image_name>  # Runs a Docker container in detached mode

docker run -p <host_port>:<container_port> <image_name>  # Runs a Docker container and maps a port from the host to the container
```

## Acknowledgements
Special thanks to the Docker community and contributors for their support and contributions to the Docker project.

## License
This guide is open-source and released under the [MIT License](/LICENSE). You are free to use and modify it for your purposes.
