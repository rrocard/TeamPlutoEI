from setuptools import find_packages, setup

package_name = 'low_level_control'

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
    maintainer='mystudent',
    maintainer_email='mystudent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pid = low_level_control.pid:main', 'speed_controller=low_level_control.speed_controller : main'
        ],
    },
)
