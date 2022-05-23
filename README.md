# rm_pioneer_vision

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/chenjunnn/rm_pioneer_vision/actions/workflows/ci.yml/badge.svg)](https://github.com/chenjunnn/rm_pioneer_vision/actions/workflows/ci.yml)

## 使用 Docker 部署 (Recommended)

1. [Install Docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)

    ```bash
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    ```

2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

    ```bash
    sudo usermod -aG docker $USER
    newgrp docker 
    ```

3. [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot)

    ```bash
    sudo systemctl enable docker.service
    sudo systemctl enable containerd.service
    ```

5. 登录 Github Container registry

    ```bash
    docker login ghcr.io
    ```
    - Username 为 Github 用户名
    - Password 为具有 `read:packages` 权限的 [PAT](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token)

3. 拉取镜像
    
    基础镜像（不含rviz2/rqt等工具）：
    ```bash
    docker pull ghcr.io/scnu-pioneer/rm_pioneer_vision:base
    ```

    桌面镜像：
    ```bash
    docker pull ghcr.io/scnu-pioneer/rm_pioneer_vision:desktop
    ```

4. 创建容器并运行
    
    基础镜像：
    ```bash
    docker run --name vision --restart always --privileged \
    -v /dev/bus/usb:/dev/bus/usb --network host \
    -v $HOME/config:/root/ros_ws/src/rm_pioneer_config \
    -e ROBOT=standard3 -it ghcr.io/scnu-pioneer/rm_pioneer_vision:base
    ```
    
    桌面镜像：
    ```bash
    docker run --name vision --restart always --privileged \
    -v /dev/bus/usb:/dev/bus/usb --network host \
    -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/config:/root/ros_ws/src/rm_pioneer_config \
    -e ROBOT=standard3 -it ghcr.io/scnu-pioneer/rm_pioneer_vision:desktop
    ```
    Tips: 
    - desktop镜像需要在图形化界面中启动容器，否则在容器内无法正常使用 GUI
    - 若需要使用 VSCode 进入容器，可在容器创建命令中加入如下行，以避免每次创建新容器时需要重新下载 vscode-server
      ```
      -v $HOME/.vision-vscode-server:/root/.vscode-server
      ```

5. 使用`exec`命令进入容器并启动 rviz2

    ```bash
    # opening up xhost to local connection
    xhost +local:
    # run zsh in vision container
    docker exec -it vision zsh
    # launch visualization
    ros2 launch rm_pioneer_bringup visualization.launch.py
    ```
    
