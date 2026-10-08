from setuptools import find_packages, setup

package_name = 'avros_webrtc'

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
    description='Low-latency teleop video: ROS image -> NVENC H.264 -> MediaMTX WebRTC',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webrtc_node = avros_webrtc.webrtc_node:main',
        ],
    },
)
