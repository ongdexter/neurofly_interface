 #!/bin/bash
docker run --privileged \
    -e DISPLAY \
    -v /dev/bus/usb:/deb/bus/usb \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /home/${USER}/.Xauthority:/home/${USER}/.Xauthority \
    -v ~/.ssh:/root/.ssh \
    -v /home/${USER}:/home/${USER} \
    -it --ipc=host --net=host --gpus all \
    --name neurofly_interface \
    --rm \
    neurofly-interface