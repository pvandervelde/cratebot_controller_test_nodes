from glob import glob
import os
from setuptools import setup


package_name = 'zinger_controller_test_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all the config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petrik',
    maintainer_email='petrikvandervelde@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publisher_joint_trajectory_controller = zinger_controller_test_nodes.publisher_joint_trajectory_controller:main",
            "publisher_velocity_controller = zinger_controller_test_nodes.publisher_velocity_controller:main",
        ],
    },
)
