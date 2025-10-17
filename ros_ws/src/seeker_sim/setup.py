# setup.py  (minimal, ROS 2 ament_python)
from setuptools import setup
from glob import glob
import os

package_name = 'seeker_sim'

def tree_as_data_files(root_dir, dest_root):
    """
    Returnerar en lista av (dest_dir, [file1, file2, ...]) poster
    så att underkataloger behålls i install/.
    """
    entries = []
    for dirpath, _, filenames in os.walk(root_dir):
        if not filenames:
            continue
        # Relativ del efter root_dir, t.ex. "Husky/meshes"
        rel = os.path.relpath(dirpath, root_dir)
        dest_dir = os.path.join(dest_root, rel) if rel != '.' else dest_root
        src_files = [os.path.join(dirpath, f) for f in filenames]
        entries.append((dest_dir, src_files))
    return entries

data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
        [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # om du har launch/
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'materials'), glob('materials/textures/*.jpg')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ]
data_files += tree_as_data_files('model', 'share/{}/model'.format(package_name))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # måste finnas en mapp seeker_sim/ med __init__.py
    data_files= data_files,

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ditt Namn',
    maintainer_email='du@example.com',
    description='Seeker simulation nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # peka på din nods main()
            'cloud_frame_relay = seeker_sim.cloud_frame_relay:main',
        ],
    },
)
