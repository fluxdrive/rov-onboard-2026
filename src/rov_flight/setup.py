import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rov_flight'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hydromeda',
    maintainer_email='hydromeda@todo.todo',
    description='ROV flight control: joystick to MAVLink, MAVROS bridge, depth hold PID',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'joy_to_mavlink = rov_flight.joy_to_mavlink:main',
        ],
    },
)
