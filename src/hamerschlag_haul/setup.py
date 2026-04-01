from setuptools import setup
from glob import glob

package_name = 'hamerschlag_haul'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhaul',
    maintainer_email='hhaul@example.com',
    description='Hamerschlag Haul autonomous delivery robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_publisher = hamerschlag_haul.odometry_publisher:main',
            'otos_odom = hamerschlag_haul.otos_odom:main',
            'vnh5019_motor_run = hamerschlag_haul.vnh5019_motor_run:main',
        ],
    },
)
