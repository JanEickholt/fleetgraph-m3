from glob import glob

from setuptools import find_packages, setup

package_name = 'fleetgraph_sim_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scenes', glob('scenes/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Eickholt',
    maintainer_email='jan.eickholt@stud.uni-due.de',
    description='FleetGraph M3 sim tools: scripted Gazebo scenes with ground truth, simulated detector per robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scene_director = fleetgraph_sim_tools.scene_director:main',
            'sim_detector = fleetgraph_sim_tools.sim_detector:main',
        ],
    },
)
