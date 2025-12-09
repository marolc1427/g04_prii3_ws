from setuptools import setup
import os
from glob import glob

package_name = 'g04_prii3_eurobot_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/worlds', ['worlds/eurobot.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marolc',
    maintainer_email='molccom@upv.edu.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eurobot_basic = g04_prii3_eurobot_turtlebot.eurobot_basic:main',
            'aruco_go_to = g04_prii3_eurobot_turtlebot.aruco_go_to:main',
        ],
    },
)