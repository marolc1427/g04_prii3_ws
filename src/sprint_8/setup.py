from setuptools import setup

package_name = 'sprint_8'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marolc',
    maintainer_email='marcosoc2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'homography_node = sprint_8.homography_node:main',
            'aruco_detector_node = sprint_8.aruco_detector_node:main',
            'webcam_node = sprint_8.webcam_node:main',
            'movimiento = sprint_8.movimiento:main',
        ],
    },
)
