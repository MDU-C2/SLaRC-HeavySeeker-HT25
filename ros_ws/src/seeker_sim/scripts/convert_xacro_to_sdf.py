import os
import subprocess


SRC_MODEL_PATH = "/slarc_ws/src/seeker_sim/model"
INSTALL_MODEL_PATH = "/slarc_ws/install/seeker_sim/share/seeker_sim/model"


os.environ["GAZEBO_MODEL_PATH"] = f"{INSTALL_MODEL_PATH}:/opt/ros/jazzy/share:{SRC_MODEL_PATH}"
os.environ["GZ_SIM_RESOURCE_PATH"] = f"{INSTALL_MODEL_PATH}::{SRC_MODEL_PATH}:{SRC_MODEL_PATH}"

print("GAZEBO_MODEL_PATH:", os.environ["GAZEBO_MODEL_PATH"])
print("GZ_SIM_RESOURCE_PATH:", os.environ["GZ_SIM_RESOURCE_PATH"])


for root, dirs, files in os.walk(SRC_MODEL_PATH):
    for file in files:
        if file.endswith(".sdf.xacro"):
            xacro_file = os.path.join(root, file)
            
            rel_path = os.path.relpath(xacro_file, SRC_MODEL_PATH)
            sdf_file = os.path.join(INSTALL_MODEL_PATH, rel_path.replace(".sdf.xacro", ".sdf"))

            
            if os.path.exists(sdf_file):
                print(f"Skipping {xacro_file}, {sdf_file} already exists")
                continue

            
            os.makedirs(os.path.dirname(sdf_file), exist_ok=True)

            print(f"Converting {xacro_file} -> {sdf_file}")
            try:
                subprocess.run([
                    "ros2", "run", "xacro", "xacro",
                    xacro_file,
                    "-o", sdf_file
                ], check=True)
            except subprocess.CalledProcessError:
                print(f"ERROR converting {xacro_file}")
