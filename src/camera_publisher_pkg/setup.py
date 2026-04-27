from setuptools import find_packages, setup

package_name = 'camera_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/launch', ['launch/camera_publisher_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'camera_publisher = camera_publisher_pkg.camera_publisher:main', 'web_streamer = camera_publisher_pkg.web_streamer:main',
        ],
    },
)
