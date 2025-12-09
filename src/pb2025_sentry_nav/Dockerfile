FROM ros:humble-ros-base

RUN sudo apt update && \
    sudo apt install python3-pip curl wget htop vim unzip -y && \
    pip install xmacro gdown

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh
CMD [ "/bin/zsh" ]

# Install small_gicp
RUN apt install -y libeigen3-dev libomp-dev && \
    mkdir -p /tmp/small_gicp && \
    cd /tmp && \
    git clone https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp

# create workspace
RUN mkdir -p ~/ros_ws && \
    cd ~/ros_ws && \
    git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git src/pb2025_sentry_nav

WORKDIR /root/ros_ws

# install dependencies and some tools
RUN rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Download simulation pcd files
RUN gdown https://drive.google.com/uc\?id\=1kAxdOU-mi1TcLssyKmDR0pz0ojF96jQ4 -O /tmp/simulation_pcd.zip && \
    unzip /tmp/simulation_pcd.zip -d /root/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/ && \
    rm /tmp/simulation_pcd.zip

# build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
source ~/ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

RUN rm -rf /var/lib/apt/lists/*
