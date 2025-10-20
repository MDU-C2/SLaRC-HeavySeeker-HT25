FROM ros:jazzy

RUN apt update && apt install -y \
    curl  \
    lsb-release \
    sudo \
    gnupg \
    build-essential \
    nano \
    neovim \
    git \
    cmake

RUN apt update && apt install -y \
    ros-dev-tools \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    ros-jazzy-rviz2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-depthai-ros

# Install gazebo harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    apt-get update \
    apt-get install gz-harmonic

# Downloand and build/install Livox-SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
RUN cd ./Livox-SDK2/ && \
    mkdir build && \
    cd build && \
    cmake .. && make -j && \
    make install -y && \
    cd ../..

# Create worksapce folder
RUN mkdir -p /slarc_ws/src

# Download and build livox_ros_driver2
RUN cd /slarc_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cd livox_ros_driver2 && \
    ./build.sh humble && \
    cd ../../..

# User ARGs
ARG USER_ID
ARG GROUP_ID
ARG USER_NAME

# Remove standard user from docker image (ros:jazzy)
RUN userdel -r $(getent passwd ${USER_ID} | cut -d: -f1) || true
RUN groupdel $(getent group ${GROUP_ID} | cut -d: -f1) || true

# Create new user
RUN groupadd -g ${GROUP_ID} ${USER_NAME} && \
    useradd -m -u ${USER_ID} -g ${USER_NAME} -s /bin/bash ${USER_NAME} && \
    usermod -aG sudo ${USER_NAME} && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Change ownership of the workspace to the new user
RUN chown -R ${USER_NAME}:${USER_NAME} /slarc_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USER_NAME}/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USER_NAME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

RUN echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
RUN echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc

ARG ROS_DOMAIN_ID=0
RUN echo "export ROS_DOMAIN_ID=" + ${ROS_DOMAIN_ID} >> ~/.bashrc

# Switch to the new user
USER ${USER_NAME}
WORKDIR /slarc_ws

ENTRYPOINT ["/bin/bash"]
