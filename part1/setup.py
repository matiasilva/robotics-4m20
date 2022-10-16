from setuptools import setup
from glob import glob

package_name = 'part1'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/configs', glob('configs/*.rviz')),
        ('share/' + package_name + '/resource', glob('resource/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jer70',
    maintainer_email='jer70@cantab.ac.uk',
    description='Mobile Robotics Course 2022-23 Part 1 Exercises',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = part1.obstacle_avoidance:main',
            'localization = part1.localization:main',
        ],
    },
)
