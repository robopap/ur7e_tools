import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur7e_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'calibration'), glob('config/calibration/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), [f for f in glob('meshes/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'meshes', 'camera_mount'), glob('meshes/camera_mount/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mines',
    maintainer_email='mines@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_joints = ur7e_tools.move_ur7e_joints:main',
            'workcell_ui = ur7e_tools.workcell_ui:main',
            'home_pose = ur7e_tools.home_pose:main',
            'gripper_visualizer = ur7e_tools.gripper_visualizer:main',
            'ft_sensor = ur7e_tools.ft_sensor_node:main',
        ],
    },
)
