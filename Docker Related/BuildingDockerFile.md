To build the Docker image from this Dockerfile, save it as Dockerfile in your project directory and run the following command:
```bash
docker build -t ibrahimmansur4/my_fyp:v2.1 .
```

Then, you can run the container with the required flags using the following command:
```bash
docker run --net=host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --device=/dev/dri:/dev/dri --group-add video --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --name=ros_ws_fyp -it ibrahimmansur4/my_fyp:v2.1
```