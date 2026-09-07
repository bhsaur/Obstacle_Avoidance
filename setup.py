import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'obst_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'gimbal_small_3d_640x480'),
            glob('models/gimbal_small_3d_640x480/*')),
        (os.path.join('share', package_name, 'models', 'iris_with_gimbal_640x480'),
            glob('models/iris_with_gimbal_640x480/*')),
        (os.path.join('share', package_name, 'models', 'textures'),
            glob('models/textures/*.png')),
    ],
    # pymavlink has no rosdep key in this environment; it's installed via
    # pip (confirmed present under ~/.local). Listed here for documentation.
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='saurabh',
    maintainer_email='prateek09bhardwaj05@gmail.com',
    description='Obstacle avoidance for the ArduPilot/Gazebo drone stack',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'avoidance_node = obst_avoidance.avoidance_node:main',
            'record_pass = obst_avoidance.record_pass:main',
            'cheap_viewer = obst_avoidance.live_viewer:main',
            'autonomous_demo = obst_avoidance.autonomous_demo:main',
        ],
    },
)
