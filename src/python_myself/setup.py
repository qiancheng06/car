from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'python_myself'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/launch", glob("launch/*.launch.py")),
        ('share/' + package_name+"/rviz", glob("rviz/*.rviz")),
        ('share/' + package_name+"/params", glob("params/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davinci-mini',
    maintainer_email='davinci-mini@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = python_myself.drive:main',
            'node1 = python_myself.node1:main',
            'node2a = python_myself.node2a:main',
            'node2b = python_myself.node2b:main',
            'node2c = python_myself.node2c:main',
            'node2d = python_myself.node2d:main',
            'node3 = python_myself.node3:main',
        ],
    },
)
