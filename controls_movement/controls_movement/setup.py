from setuptools import find_packages, setup

package_name = 'controls_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moveLeft = controls_core.move_test:moveLeft",
            "moveRight = controls_core.move_test:moveRight",
            "moveFront = controls_core.move_test:moveFront",
            "moveBack = controls_core.move_test:moveBack",
            "moveUp = controls_core.move_test:moveUp",
            "moveDown = controls_core.move_test:moveDown",
        ],
    },
)
