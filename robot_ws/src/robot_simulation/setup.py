from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/urdf', glob('urdf/*.trans')),
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*.stl')),
    ],
    install_requires=['numpy', 'pybullet', 'scipy', 'setuptools'],
    zip_safe=True,
    maintainer='Mostafa Hussein',
    maintainer_email='mahussein04@gmail.com',
    description='Deterministic PyBullet simulation and idealized ROS 2 teaching sensors.',
    license='NOASSERTION',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_sim = robot_simulation.pybullet_sim:main',
            'state_interface_monitor = '
            'robot_simulation.state_interface_monitor:main',
        ],
    },
)
