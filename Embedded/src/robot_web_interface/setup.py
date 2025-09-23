import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='donggun',
    maintainer_email='dinodragon@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_status_client = robot_web_interface.robot_status_client:main',
            'robot_status_monitor = robot_web_interface.robot_status_monitor:main',
            'turtlebot_status_monitor = robot_web_interface.turtlebot_status_monitor:main',
            'delivery_address_client = robot_web_interface.delivery_address_client:main',
            'robot_http_server = robot_web_interface.robot_http_server:main',
        ],
    },
)
