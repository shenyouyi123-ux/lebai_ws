from glob import glob
from setuptools import find_packages, setup

package_name = 'lebai_vision_grasp'

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
    description='Vision-guided grasp orchestrator for Lebai LM3 using existing MoveIt and gripper bridge.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_grasp_orchestrator = lebai_vision_grasp.vision_grasp_orchestrator:main',
            'fake_target_position = lebai_vision_grasp.fake_target_position:main',
            'gripper_sim_node = lebai_vision_grasp.gripper_sim_node:main',
        ],
    },
)
