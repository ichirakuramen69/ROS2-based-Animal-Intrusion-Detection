from setuptools import setup
from glob import glob
import os

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Install index.html into share/rover_control/
        ('share/' + package_name, ['index.html']),
    ],
    install_requires=['setuptools', 'flask', 'pyserial'],
    zip_safe=True,
    maintainer='rasp',
    maintainer_email='rasp@todo.todo',
    description='Rover control web server',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rover_control = rover_control.web_server:main',
        ],
    },
)
