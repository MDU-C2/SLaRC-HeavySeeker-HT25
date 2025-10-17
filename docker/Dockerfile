FROM ros:jazzy

RUN apt update && apt install -y \
    curl  \
    lsb-release \
    sudo \
    gnupg \
    build-essential \
    nano \
    neovim

RUN apt update && apt install -y \
    ros-dev-tools \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    ros-jazzy-rviz2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-depthai-ros

# Install gazebo harmonic
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
#     apt-get update \
#     apt-get install gz-harmonic

# 1) Base tools + add OSRF repo
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release ca-certificates \ 
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/gazebo-stable.list \
&& apt-get update

# 2) Install GZ (Harmonic). No commas; add useful tools/GUI as needed.
RUN apt-get install -y --no-install-recommends \
    gz-harmonic gz-sim gz-tools gz-gui \
&& rm -rf /var/lib/apt/lists/*

# 3) (Option A) Pre-download “standard” models from Fuel into the image
# You can add more models similarly.
RUN mkdir -p /root/.gz/fuel \
&& gz fuel download --url "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane" \
&& gz fuel download --url "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun"

# 4) Make Fuel cache available by default + leave room to append your own project path
ENV GZ_SIM_RESOURCE_PATH="/root/.gz/fuel"


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
RUN echo "export ROS_DOMAIN_ID=" + ${ROS_DOMAIN_ID} >> /home/${USER_NAME}/.bashrc

# Switch to the new user
USER ${USER_NAME}
WORKDIR /slarc_ws

ENTRYPOINT ["/bin/bash"]
