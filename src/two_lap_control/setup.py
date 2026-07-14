from setuptools import setup

package_name = 'two_lap_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/car_go.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Two Lap User',
    maintainer_email='you@example.com',
    description='Two-lap CV + LiDAR control nodes extracted from racecar package.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_follow_node = two_lap_control.cv_follow_node:main',
            'car_test_node = two_lap_control.car_test_node:main',
        ],
    },
)
