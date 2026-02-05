from setuptools import setup
import os
from glob import glob

package_name = 'omni_drive'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='四轮正交全向底盘ROS2驱动（集成CAN接口配置）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omni_drive_node = omni_drive.omni_drive_node:main',
             'test = omni_drive.test_velocity:main',
             'omni = omni_drive.omni:main'
            
        ],
    },
)