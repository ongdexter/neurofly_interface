 #!/bin/bash
docker run --privileged \
    -e DISPLAY \
    -v /dev:/dev \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /home/${USER}/.Xauthority:/home/${USER}/.Xauthority \
    -v ~/.ssh:/root/.ssh \
    -v /home/${USER}:/home/${USER} \
    -v /usr/local/zed/resources:/usr/local/zed/resources \
    -it --ipc=host --net=host --gpus all \
    --name neurofly_interface \
    neurofly-interface
