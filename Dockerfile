FROM endermands/ros_noetic_zsh:opencv

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m
ARG PROJECT_NAME=orbbec_competition

# install binary
RUN sudo apt update && \
    sudo apt install -y ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros && \
    sudo rm -rf /var/lib/apt/lists/*

# compile project
WORKDIR /home/$USERNAME/code/ros_ws
COPY . /home/$USERNAME/code/ros_ws
RUN sudo chmod 777 -R /home/$USERNAME/code && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release && \
    echo "source /home/${USERNAME}/code/ros_ws/devel/setup.zsh" >> /home/${USERNAME}/.zshrc

ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/home/${USERNAME}/code/ros_ws/start.zsh" ]
