from setuptools import setup

package_name = 'servo_sweeper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasp',
    maintainer_email='rasp@todo.todo',
    description='Servo sweeping node with auto reverse',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'servo_sweeper = servo_sweeper.sweeper_node:main',
        ],
    },
)
