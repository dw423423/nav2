from setuptools import find_packages, setup
from glob import glob
package_name = 'pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/launch", glob('launch/*.launch.py')),
        ('share/' + package_name+"/config", ['config/pose_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='union',
    maintainer_email='union@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "pose_node = pose.Nav2_Pose:main"
            ,"goal_node = pose.goal:main"
            ,"v1_node = pose.v1:main"
            ,"v2_node = pose.v2:main"
            ,"speech_recognition_node = pose.speech_recognition:main"
        ],
    },
)
