from setuptools import find_packages, setup

package_name = 'jabari_dht11_control_pkg'

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
             "fan_controller_node = jabari_dht11_control_pkg.fan_controller_node:main",
              "dht11_node = jabari_dht11_control_pkg.dht11_node:main",
              "mock_adafruit_dht = jabari_dht11_control_pkg.mock_adafruit_dht:main",
        ],
    },
)
