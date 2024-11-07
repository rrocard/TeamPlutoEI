from setuptools import find_packages, setup

package_name = 'advanced_behaviors'

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
        'console_scripts': ['slide = advanced_behaviors.slide : main', 'u_turn =advanced_behaviors.u_turn : main'
        ],
    },
)
