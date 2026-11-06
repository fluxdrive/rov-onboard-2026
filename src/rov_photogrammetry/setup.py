from setuptools import find_packages, setup

package_name = 'rov_photogrammetry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hydromeda',
    maintainer_email='hydromeda@todo.todo',
    description='Onboard photogrammetry: Pi Zero camera bridge (preview + capture)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'node = rov_photogrammetry.node:main',
        ],
    },
)
