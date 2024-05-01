# Pre-Requisite Packages

#### Install the following packages before starting the setup.

```bash
git clone https://github.com/reedhedges/AriaCoda

git clone https://github.com/amor-ros-pkg/rosaria

git clone https://github.com/pengtang/rosaria_client.git

```

## For p3dx robot interfacing with laptop

- Start ros master
```bash
roscore   
```
- List all available usb devices connected
```bash
ls /dev/tty*  

sudo chmod 777 /dev/ttyUSB0
```
- Connect to rosaria

```bash
rosrun rosaria Rosaria   
```
- Run teleop node
```bash
rosrun rosaria_client teleop 
```
- Run sonar node
```bash
rosrun rosaria sonar  
```
