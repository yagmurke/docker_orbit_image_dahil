import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rviz2py'
connectors = f'{package_name}/connectors'
rviz2_visualizer = f'{package_name}/rviz2_visualizer'
rviz2_threads = f'{package_name}/rviz2py_threads'
rviz2_scene_items = f'{package_name}/rviz2_scene_items'
rviz2_styles = f'{package_name}/rviz2_styles'
utils = f'{package_name}/utils'


setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name, 
        connectors, 
        rviz2_visualizer, 
        rviz2_threads, 
        rviz2_scene_items,
        rviz2_styles,
        utils
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource', 'images'), glob('resource/images/*')),
        (os.path.join('share', package_name, 'maps', 'keepout'), glob('maps/keepout/*.pgm')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*',)),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ovali',
    maintainer_email='ovali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
