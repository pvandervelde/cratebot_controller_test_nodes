from setuptools import setup

package_name = 'cratebot_controller_test_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "publisher_joint_trajectory_controller = cratebot_controller_test_nodes.publisher_joint_trajectory_controller:main",
            "publisher_velocity_controller = cratebot_controller_test_nodes.publisher_velocity_controller:main",
        ],
    },
)
