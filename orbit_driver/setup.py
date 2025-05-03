import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'orbit_driver'
zlac8015d = f'{package_name}/zlac8015d_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, zlac8015d],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalebu',
    maintainer_email='calebndatimana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "orbit_driver_nodee=orbit_driver.orbit_driver_node_v11:main",
            "batterry_smooth_node = orbit_driver.batterry_smooth:main",
            "imu_status_node = orbit_driver.imu_status:main",
        ],
    },
)
