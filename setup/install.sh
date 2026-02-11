#! /bin/bash

# General Install / Config
echo "Installing general/basic packages"

sudo apt update && sudo apt upgrade -y
sudo apt install -y ufw git ssh openssh-server bash-completion
sudo apt install -y nano neovim wget curl tmux python-is-python3
sudo apt install -y gnupg2 lsb-release build-essential software-properties-common bluez
# add pip3?

# Firewall configuration
echo "Configuring firewall"
sudo ufw allow ssh
sudo ufw allow 7447 # Zenoh
#sudo ufw allow xxx # lidar???
sudo ufw enable

# open SSH
echo "Configure openssh server"
sudo systemctl enable ssh
sudo systemctl start ssh

# tmux
echo "configuring tmux"
cd ~
git clone --single-branch https://github.com/gpakosz/.tmux.git
ln -s -f .tmux/.tmux.conf
cp .tmux/.tmux.conf.local .

# nvim
echo "configuring nvim"
if ! grep -q "alias vim=nvim" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "alias vim=nvim" >> ~/.bashrc
fi

# install video driver
sudo apt update && sudo apt install -y intel-media-va-driver-non-free vainfo


# Install ROS2-Jazzy
echo "Installing ROS2-Jazzy"

sudo apt install 
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-dev-tools ros-jazzy-ros-base
#sudo apt install -y ros-jazzy-usb-cam #ros-jazzy-depthai-ros-driver
sudo apt install -y python3-argcomplete python3-colcon-clean python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo rosdep init
rosdep update

echo '' >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc
echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> ~/.bashrc

echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc

# Install Zenoh
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
sudo apt update
sudo apt install zenoh -y

# Platform Requirements
./install_sensor_requirements.sh

# enable kernel module for can
sudo modprobe --all can can_raw vcan


# Install Zerotier 
curl -s https://install.zerotier.com | sudo bash

# source this instance
source ~/.bashrc

# Info to user
echo "Installation end sucessfully"
echo "Remember to connect to zerotier network and allow zerotier network traffic trought ufw"