import os

from setuptools import find_packages, setup


package_name = 'avros_data_logger'

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
    maintainer='AV Lab',
    maintainer_email='avlab@cpp.edu',
    description='Synchronized RGB, thermal, and vehicle-state data logging',
    license='MIT',
    entry_points={
        'console_scripts': [
            'avros_dataLogger = avros_data_logger.data_logger_node:main',
        ],
    },
)
