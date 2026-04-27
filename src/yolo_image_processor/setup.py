from setuptools import find_packages, setup
import os
import glob

package_name = 'yolo_image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLO Image Processor for ROS2 using OpenCV and YOLOv8',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolo_image_processor = yolo_image_processor.object_detection:main',
        ],
    },
)


