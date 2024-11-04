from setuptools import setup

package_name = 'twist2pic'

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
            'twist2pic = twist2pic.twist2pic:main'  # Points to main() in twist2pic/twist2pic.py
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/twist2piclaunch.py']),
    ],
)
