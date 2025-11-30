from setuptools import setup
import os
from glob import glob

package_name = 'companion_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 bridge for Elderly Companion Robot AI services',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_bridge = companion_bridge.nodes.ai_bridge:main',
            'behavior_manager = companion_bridge.nodes.behavior_manager:main',
        ],
    },
)

