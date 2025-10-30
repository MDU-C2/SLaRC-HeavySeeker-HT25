import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def convert_xacro_to_sdf(model_name: str) -> str:

    model_dir = os.path.join(
        get_package_share_directory("seeker_sim"), "model", "Assemblies", model_name
    )
    xacro_file = os.path.join(model_dir, "model.sdf.xacro")
    sdf_file = os.path.join(model_dir, "model.sdf")

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    if not os.path.exists(sdf_file):
        print(f"[INFO] Converting {xacro_file} → {sdf_file}")

        urdf_temp = os.path.join(model_dir, "model.urdf")
        subprocess.run(["xacro", xacro_file, "-o", urdf_temp], check=True)

        with open(sdf_file, "w") as sdf_out:
            subprocess.run(["gz", "sdf", "-p", urdf_temp], stdout=sdf_out, check=True)

        os.remove(urdf_temp)
    else:
        print(f"[INFO] SDF already exists at {sdf_file}")

    return sdf_file
