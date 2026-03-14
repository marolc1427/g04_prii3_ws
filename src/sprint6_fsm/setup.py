from setuptools import setup

package_name = 'sprint6_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transitions'],
    zip_safe=True,
    maintainer='marolc',
    maintainer_email='marcosoc2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_sprint6 = sprint6_fsm.fsm_sprint6:main',
            'test = sprint6_fsm.test:main',
            'prueba = sprint6_fsm.prueba:main',
        ],
    },
)
