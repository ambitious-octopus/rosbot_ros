xhost +local:root
docker build -t rosbot-humble -f tools/Dockerfile.dev .
docker run -it -v $(pwd):/workspace --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=unix$DISPLAY --device /dev/dri -v /home/$USER/.Xauthority:/root/.Xauthority rosbot-humble bash