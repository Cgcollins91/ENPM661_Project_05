# Project: ENPM662-Project1-Group1
# Description: Humblebot Dockerfile

FROM osrf/ros:humble-desktop-full

#expecting these to be set using build-args
ARG USER_ID
ARG GROUP_ID
ARG USERNAME
ARG WSLG
ARG HOST_DISPLAY

# set display using existing display
ENV DISPLAY=${HOST_DISPLAY}
ENV WAYLAND_DISPLAY=wayland-0
# if WSL, set this to /mnt/wslg/runtime-dir/ using build-args
ENV XDG_RUNTIME_DIR=${WSLG}

# package updates and installs
RUN apt-get update && apt-get upgrade -y
# bare minimum packages
RUN apt-get install -y curl file git less locales net-tools rsync sudo unzip vim wget
# quality of life packages
RUN apt-get install -y jq fd-find gettext htop iputils-ping iproute2 lsb-release neofetch ripgrep xclip zstd
# C/C++ development packages
RUN apt-get install -y bison build-essential clang clang-format clangd clang-tidy cmake flex g++ gdb make
# Python development packages
RUN apt-get install -y python3 python3-pip python3-venv python3-dev
# Javascript development packages
RUN apt-get install -y npm nodejs

# locale config
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# fixup the .bashrc with some aliases and environment variables
COPY assets/helpful.bash /tmp/helpful.bash
RUN cat /tmp/helpful.bash >> /root/.bashrc

# create user account
RUN groupadd -g ${GROUP_ID} ${USERNAME} 
RUN useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME}
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# add a few helpful items to bash and colcon config
COPY assets/helpful.bash /tmp/helpful.bash
RUN cat /tmp/helpful.bash >> .bashrc
RUN mkdir -p .config/colcon
COPY assets/colcon.yaml .config/colcon/colcon.yaml

# install and configure neovim
RUN cd .config && git clone https://github.com/jamesfehrmann/nvim.git
RUN git clone https://github.com/neovim/neovim && cd neovim && git checkout stable
RUN cd neovim && make CMAKE_BUILD_TYPE=RelWithDebInfo
RUN cd neovim/build && cpack -G DEB && sudo dpkg -i nvim-linux-x86_64.deb
RUN rm -rf neovim

# install ROS2 and turtlebot
RUN sudo mkdir -p /turtlebot/src
RUN sudo chown -R ${USERNAME}:${USERNAME} /turtlebot
WORKDIR /turtlebot
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
RUN vcs import src<turtlebot3.repos
RUN rosdep update && rosdep install --from-paths src --ignore-src -y 
RUN . /opt/ros/humble/setup.sh && colcon build #do not split these

# add helpful ROS/Gazebo scripts
COPY assets/run_world.sh /turtlebot/run_world.sh
COPY assets/run_emptyworld.sh /turtlebot/run_emptyworld.sh
COPY assets/run_teleop.sh /turtlebot/run_teleop.sh

# for project1
RUN sudo apt-get install -y ros-humble-ros2-control
RUN sudo apt-get install -y ros-humble-ros2-controllers
RUN sudo apt-get install -y ros-humble-gazebo-ros2-control
RUN sudo apt-get install -y ros-humble-joint-state-publisher
RUN sudo apt-get install -y ros-humble-controller-manager
RUN sudo apt-get install -y ros-humble-joint-state-publisher-gui
RUN pip install pynput
