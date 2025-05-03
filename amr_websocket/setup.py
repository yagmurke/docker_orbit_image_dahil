import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'amr_websocket'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource', 'temp'), glob('resouce/temp/*'))
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
            'action_server = amr_websocket.action_server:main',
            'tf_listener = amr_websocket.tf_listener:main',
            'sensor_control = amr_websocket.sensor_control:main',
            'file_sender = amr_websocket.file_sender:main',
            'auto_dock = amr_websocket.auto_dock:main',
            'delivery_control = amr_websocket.delivery_control:main',
            'map_publisher = amr_websocket.map_publisher:main',
            'keepout_point_publisher = amr_websocket.keepout_point_publisher:main',
            'path_publisher = path_reducer.path_publisher:main'
            'amcl_estimate = amr_websocket.amcl_estimate:main'
        ],
    },
)
