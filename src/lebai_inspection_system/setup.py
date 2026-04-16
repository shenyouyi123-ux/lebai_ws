from glob import glob
from setuptools import find_packages, setup

package_name = 'lebai_inspection_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhengshaolong',
    maintainer_email='shenyouyi123@gmail.com',
    description='Non-invasive inspection orchestration package for Lebai MoveIt setup.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_orchestrator = lebai_inspection_system.task_orchestrator_node:main',
            'fake_perception = lebai_inspection_system.fake_perception_node:main',
            'inspection_logger = lebai_inspection_system.inspection_logger_node:main',
        ],
    },
)
