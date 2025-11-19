from setuptools import setup, find_packages

package_name = 'telegram_alert'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasp',
    maintainer_email='rasp@todo.todo',
    description='Telegram alert system for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'telegram_alert = telegram_alert.alert_node:main',
        ],
    },
)
