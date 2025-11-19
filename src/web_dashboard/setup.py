from setuptools import setup

package_name = 'web_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', ['web_dashboard/templates/index.html']),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='rasp',
    maintainer_email='rasp@todo.todo',
    description='Web Dashboard for camera stream',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'web_dashboard = web_dashboard.dashboard_node:main',
        ],
    },
)
