import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'orbit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ovali',
    maintainer_email='calebndatimana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orbit_movement = orbit_controller.orbit_movement_tf_v2:main',
            'test = orbit_controller.test:main',
            'bs64totext_node = orbit_controller.bs64totext:main',
        ],
    },
)
