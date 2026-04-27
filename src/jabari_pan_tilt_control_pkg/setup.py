from setuptools import find_packages, setup

package_name = 'jabari_pan_tilt_control_pkg'

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
    maintainer='libes',
    maintainer_email='libes@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "servo_node = jabari_pan_tilt_control_pkg.servo_node:main",
            "keyboard_servo_control_node = jabari_pan_tilt_control_pkg.keyboard_servo_control_node:main",
            "mock_rpi_global = jabari_pan_tilt_control_pkg.mock_rpi_global:main",
            "mock_gpio = jabari_pan_tilt_control_pkg.mock_gpio:main",
        ],
    },
)
