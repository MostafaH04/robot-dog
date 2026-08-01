from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['numpy', 'setuptools'],
    zip_safe=True,
    maintainer='Mostafa Hussein',
    maintainer_email='mahussein04@gmail.com',
    description='ROS 2 joint command adapter and leg kinematics for the robot dog.',
    license='NOASSERTION',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_controller = robot_controller.quad_joint_controller:main',
        ],
    },
)
