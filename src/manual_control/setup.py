from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
            
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json')),
            
        # Install documentation
        (os.path.join('share', package_name, 'docs'),
            glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mwaura',
    maintainer_email='nathanwaweru75@gmail.com',
    description='Manual control node for JABARI Mars Rover',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'manual_control_node = manual_control.manual_control_node:main',
            'test_manual_control = manual_control.test_manual_control:main',
        ],
    },
     classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Robotics',
        'Operating System :: POSIX :: Linux',
    ],
    python_requires='>=3.8',
)
