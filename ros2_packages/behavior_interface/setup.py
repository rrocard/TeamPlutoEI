from setuptools import setup
import os
from glob import glob

package_name = 'behavior_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for managing behaviors',
    license='Apache License 2.0',
sqf    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_behavior_a = behavior_interfac.base_behavior:fake_behavior_a',
            'fake_behavior_b = behavior_interface.base_behavior:fake_behavior_b',
        ],
    },
)
