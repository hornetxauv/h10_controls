from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controls_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. (includes both python and xml launch files)
        (os.path.join("share", package_name, "launch"), glob("launch_files/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ControlsSubteam',
    maintainer_email='izentoh.zenith@gmail.com',
    description='Pool test attempt 1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moveLeft = controls_movement.pool_test_2:main",
            "moveRight = controls_core.move_test:moveRight",
            "moveFront = controls_core.move_test:moveFront",
            "moveBack = controls_core.move_test:moveBack",
            "moveUp = controls_core.move_test:moveUp",
            "moveDown = controls_core.move_test:moveDown",

            "vertPID = controls_movement.vertical_PID_test:main",
            "vertPID_pub = controls_movement.vertical_PID_sample_pub:main",

            "uturn = controls_movement.uturn_test:main",

            "qualiGate = controls_movement.quali_thruster_allocator:main",
            "qualiGate_pub = controls_movement.quali_thruster_sample_pub:main",

            "dirTest = controls_movement.simple_direction_test:main",

            "depthOriPID = controls_movement.depth_orientation_pid:main",
            "movementControls = movement_controller:main"
        ],
    },
)
