import os
from glob import glob
from setuptools import setup

package_name = 'dual_arm_teleoperation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # RViz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Meshes - Visual
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.dae')),
        # Meshes - Collision
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanhbao',
    maintainer_email='thanhbao@example.com',
    description='Dual UR5 Teleoperation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_arm_controller = dual_arm_teleoperation.dual_arm_controller:main',
            'hand_teleop_node = dual_arm_teleoperation.hand_teleop_node:main',
            'keyboard_teleop = dual_arm_teleoperation.keyboard_teleop:main',
        ],
    },
)
