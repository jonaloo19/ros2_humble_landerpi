from setuptools import setup
import os
from glob import glob

package_name = 'landerpi_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi Developer',
    maintainer_email='user@todo.todo',
    description='LanderPi robotic arm manipulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_action_server = landerpi_arm.grasp_action_server:main',
            'send_pick_goal = landerpi_arm.send_pick_goal:main',
            'color_pick_executor = landerpi_arm.color_pick_executor:main',
        ],
    },
)

