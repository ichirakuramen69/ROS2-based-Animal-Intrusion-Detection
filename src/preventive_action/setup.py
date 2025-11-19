from setuptools import setup

package_name = 'preventive_action'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Preventive alarm node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'preventive_action = preventive_action.preventive_action_node:main',
        ],
    },
)
