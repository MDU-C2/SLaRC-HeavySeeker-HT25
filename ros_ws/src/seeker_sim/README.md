Steps to run UGV (Husky) with LiDAR (Livox mid360) in Gazebo and rviz2.

1. Install the dependencies

$ sudo apt update
# Gnome terminal (om du inte redan har det)
$ sudo apt install gnome-terminal

# ROS 2 Jazzy (om du inte redan har det)
$ sudo apt install ros-jazzy-desktop

# Gazebo (Harmonic via ROS vendor) + ros_gz
sudo apt install ros-jazzy-ros-gz

# RViz2
sudo apt install ros-jazzy-rviz2

2. Build & Source
$ colcon build && source install/setup.bash


3. Run the simulation
$ ros2 launch seeker_sim seeker_sim.launch.py

4. Run simulation with arguments (optional)

$ ros2 launch seeker_sim seeker_sim.launch.py --show -args
And follow the description there.

