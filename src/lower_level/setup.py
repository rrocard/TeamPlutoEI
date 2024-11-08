from setuptools import setup
import os
from glob import glob

package_name = 'lower_level'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Specifies the Python package directory
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # Replace with your name
    maintainer_email='your_email@example.com',  # Replace with your email
    description='A package to visualize Twist commands using OpenCV',
    license='License declaration',  # Replace with your license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist2pic = lower_level.twist2pic:main',  # Points to main() in lower_level/twist2pic.py
            'joyteleop = lower_level.joyteleop:main',  # Points to main() in lower_level/joyteleop.py
            'speed_controller = lower_level.speed_controller:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
)
