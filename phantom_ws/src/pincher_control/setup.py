from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pincher_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joho',
    maintainer_email='johalopezari@unal.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'template_control_servo = pincher_control.template_control_servo:main',
            'control_servo = pincher_control.control_servo:main',
            'follow_joint_trajectory = pincher_control.follow_joint_trajectory_node:main',
            'joint_angles_degrees = pincher_control.joint_angles_degrees:main',
            'object_sorting = pincher_control.object_sorting_node:main',
            'shape_to_pose = pincher_control.shape_to_pose_node:main',
            'clasificador = pincher_control.clasificador_node:main',
            'pose_search = pincher_control.pose_search_node:main',
            'routine_manager = pincher_control.routine_manager:main',
        ],
    },
)
