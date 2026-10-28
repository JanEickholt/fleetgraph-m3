from setuptools import find_packages, setup

package_name = 'fleetgraph_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Eickholt',
    maintainer_email='jan.eickholt@stud.uni-due.de',
    description='FleetGraph M3 robot agent: task state machine, Nav2 execution, telemetry; mock orchestrator.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agent = fleetgraph_robot.agent_node:main',
            'mock_orchestrator = fleetgraph_robot.mock_orchestrator:main',
        ],
    },
)
