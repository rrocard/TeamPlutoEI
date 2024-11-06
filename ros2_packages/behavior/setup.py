from setuptools import find_packages, setup

package_name = 'behavior'

node_scripts = ['takeoff = behavior.behaviors:takeoff', 'land = behavior.behaviors:land', 
             'hover = behavior.behaviors:hover',
             'status_viewer = behavior.status_viewer:main',
             'joy_teleop = behavior.joy_teleop:main', 'command = behavior.command:main', ]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st5dronelab',
    maintainer_email='raphael.rocard@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': node_scripts,
    },
)
