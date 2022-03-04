FROM osrf/ros:galactic-desktop

# setup openvino
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    wget 

RUN wget https://apt.repos.intel.com/openvino/2020/GPG-PUB-KEY-INTEL-OPENVINO-2020 && \
    apt-key add GPG-PUB-KEY-INTEL-OPENVINO-2020

RUN echo "deb https://apt.repos.intel.com/openvino/2021 all main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2021.list

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    intel-openvino-runtime-ubuntu20-2021.4.752 \
    && rm -rf /var/lib/apt/lists/*

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t jispwoso \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting \
    && rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws/

# copy source code
COPY . src/

# install dependencies
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# build source
SHELL [ "/bin/zsh", "-c" ]
RUN source /opt/ros/galactic/setup.zsh && \
    source /opt/intel/openvino_2021/bin/setupvars.sh && \
    colcon build \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --symlink-install

RUN echo \
    $'export TERM=xterm-256color \n\
    source /opt/intel/openvino_2021/bin/setupvars.sh \n\
    source /root/ros_ws/install/setup.zsh \n\
    eval "$(register-python-argcomplete3 ros2)" \n\
    eval "$(register-python-argcomplete3 colcon)"' >> ~/.zshrc

ENV ROBOT=guard

CMD [ "src/startup.sh" ]