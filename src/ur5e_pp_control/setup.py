from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5e_pp_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed',
    maintainer_email='hamoudaahmad8@gmail.com',
    description='UR5e pick-and-place controller comparison in Gazebo',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'pp_runner = ur5e_pp_control.runner_node:main',
            'validate = ur5e_pp_control.validate:main',
            'energy_monitor = ur5e_pp_control.energy_monitor:main',
            'learn_paths = ur5e_pp_control.runner_node:learn_offline',
        ],
    },
)
