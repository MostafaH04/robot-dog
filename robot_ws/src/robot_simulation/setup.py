from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', ]),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name + "/rviz", glob('rviz/*.rviz')),
        ('share/' + package_name + "/urdf", glob('urdf/*.xacro')),
        ('share/' + package_name + "/urdf", glob('urdf/*.trans')),
        ('share/' + package_name + "/urdf/meshes", glob('urdf/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahussein',
    maintainer_email='mahussein04@gmail.com',
    description='Quadruped Simulation in Pybullet package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_sum = robot_simulation.pybullet_sim:main',
        ],
    },
)