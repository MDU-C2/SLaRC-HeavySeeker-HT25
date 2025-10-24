FROM ros:jazzy

RUN apt update && apt install -y \
    curl  \
    lsb-release \
    sudo \
    gnupg \
    build-essential \
    nano \
    neovim \
    gh

RUN apt update && apt install -y \
    ros-dev-tools \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    ros-jazzy-rviz2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-depthai-ros \
    ros-jazzy-septentrio-gnss-driver \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy

# Install gazebo harmonic and gz-sensors
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get update && \
    sudo apt-get install -y libgz-sensors10-navsat libgz-sensors10-navsat-dev gz-harmonic


# Create worksapce folder
RUN mkdir /slarc_ws

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
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/${USER_NAME}/.bashrc

RUN echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /home/${USER_NAME}/.bashrc
RUN echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/${USER_NAME}/.bashrc



ARG ROS_DOMAIN_ID=0
RUN echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /home/${USER_NAME}/.bashrc

# Switch to the new user
USER ${USER_NAME}
WORKDIR /slarc_ws

ENTRYPOINT ["/bin/bash"]
