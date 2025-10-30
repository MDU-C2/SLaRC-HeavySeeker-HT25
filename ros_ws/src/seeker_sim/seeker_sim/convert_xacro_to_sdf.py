import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def convert_xacro_to_sdf(model_name: str) -> str:

    model_dir = os.path.join(get_package_share_directory("seeker_sim"),"model","Assemblies",model_name)
    xacro_file_path = os.path.join(model_dir, "model.sdf.xacro")
    sdf_file_path   = os.path.join(model_dir, "model.sdf")

    # Safety: check if we have a sdf.xacro file
    if not os.path.exists(xacro_file_path):
        raise FileNotFoundError(f"[ERROR] Xacro file not found: {xacro_file_path}")

    
    
    print(f"[INFO] Converting {xacro_file_path} → {sdf_file_path}")

    # Run xacro directly on the SDF.xacro and write result to model.sdf
    # This is basically: xacro model.sdf.xacro -o model.sdf
    with open(sdf_file_path, "w") as sdf_out:
        subprocess.run(
            ["xacro", xacro_file_path],
            check=True,
            stdout=sdf_out
        )

    print(f"[INFO] Done. Wrote {sdf_file_path}")

    return sdf_file_path

