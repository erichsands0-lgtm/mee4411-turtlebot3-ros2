import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cnn_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Philip Dames',
    maintainer_email='pdames@temple.edu',
    description='Create a CNN to control the motion of a robot',
    license='LGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cnn_controller_node = cnn_controller.cnn_controller_node:main',
        ],
    },
)
