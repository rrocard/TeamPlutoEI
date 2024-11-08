from setuptools import find_packages, setup

package_name = 'vanishing'

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
    maintainer='2023wangch',
    maintainer_email='christophe.wang@student-cs.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['vp = vanishing.vp_node:main','shift = vanishing.shift_node:main',
        'controller=vanishing.controller:main'
        ],
    },
)
