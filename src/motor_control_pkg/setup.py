from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samkiche',
    maintainer_email='samkiche@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            "motor_controller_node = motor_control_pkg.motor_controller_node:main",
            "keyboard_control_node = motor_control_pkg.keyboard_control_node:main",
            "dual_motor_driver_node = motor_control_pkg.dual_motor_driver_node:main"
        ],
    },
)
