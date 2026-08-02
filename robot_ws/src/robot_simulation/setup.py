from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['models/*.xml']},
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
    install_requires=[
        'mujoco==3.6.0',
        'numpy',
        'pybullet',
        'scipy',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Mostafa Hussein',
    maintainer_email='mahussein04@gmail.com',
    description='Deterministic teaching simulators with explicit sensor/truth boundaries.',
    license='NOASSERTION',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_sim = robot_simulation.pybullet_sim:main',
            'mujoco_smoke = robot_simulation.mujoco_smoke:main',
            'state_interface_monitor = '
            'robot_simulation.state_interface_monitor:main',
        ],
    },
)
