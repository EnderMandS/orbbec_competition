FROM endermands/ros_noetic_zsh:opencv

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m
ARG PROJECT_NAME=orbbec_competition

# install binary
RUN sudo apt update && \
    sudo apt install -y ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros && \
    sudo apt install -y libx264-dev \
    sudo rm -rf /var/lib/apt/lists/*

# compile project
WORKDIR /home/${USERNAME}/code/ros_ws
COPY . /home/${USERNAME}/code/ros_ws
RUN sudo rm -rf IsaacAsset/ && sudo rm -rf README/ && sudo rm README.md && \
    sudo chmod 777 -R /home/${USERNAME}/code && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release && \
    echo "source /home/m/code/ros_ws/devel/setup.zsh" >> /home/${USERNAME}/.zshrc

# Images taken by the program are saved to this directory
VOLUME [ "/home/m/code/ros_ws" ]

WORKDIR /home/${USERNAME}/code/ros_ws/src/isaac_tf/launch

# ENTRYPOINT [ "/bin/zsh" ]
ENTRYPOINT [ "/home/m/code/ros_ws/start.zsh" ]
