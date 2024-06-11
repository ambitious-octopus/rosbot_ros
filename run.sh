xhost +local:root
docker build -t rosbot-humble .
docker run -it -v $(pwd):/workspace --gpus all --net host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=unix$DISPLAY --device /dev/dri --privileged -v /home/$USER/.Xauthority:/root/.Xauthority rosbot-humble bash