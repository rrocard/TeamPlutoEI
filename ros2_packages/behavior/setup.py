from setuptools import find_packages, setup

package_name = 'behavior'

node_scripts = ['fake_behavior_a = behavior.base_behavior:fake_behavior_a',
            'fake_behavior_b = behavior.base_behavior:fake_behavior_b']

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st5dronelab',
    maintainer_email='emileandrieu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': node_scripts,
    },
)
