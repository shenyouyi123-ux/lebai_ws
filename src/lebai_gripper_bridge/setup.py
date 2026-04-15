from setuptools import setup

package_name = 'lebai_gripper_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shenyouyi',
    maintainer_email='shenyouyi123@gmail.com',
    description='Gripper bridge: FollowJointTrajectory -> SetGripper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_bridge = lebai_gripper_bridge.gripper_bridge_node:main',
        ],
    },
)
