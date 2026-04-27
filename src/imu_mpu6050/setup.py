from setuptools import setup

package_name = 'imu_mpu6050'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='clintonsabi283@gmail.com',
    description='ROS2 driver node for MPU6050 IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "imu_mpu6050_node = imu_mpu6050.imu_mpu6050_node:main"
        ],
    },
)

