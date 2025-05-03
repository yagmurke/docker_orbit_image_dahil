import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'play_face'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource', 'videos'), glob('resource/videos/*.mp4')),
        (os.path.join('share', package_name, 'resource', 'videos', 'ads'), glob('resource/videos/ads/*.mp4')),
        (os.path.join('share', package_name, 'resource', 'images', 'faces'), glob('resource/images/faces/*.png')),
        (os.path.join('share', package_name, 'resource', 'images', 'lips'), glob('resource/images/lips/*.png')),
        (os.path.join('share', package_name, 'resource','audios'), glob('resource/audios/*.mp3',)),
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
            'play_face_node = play_face.play_face_node:main',
            'mp3_amp_publisher = play_face.mp3_amp_publisher:main',
            'text_amp_publisher = play_face.text_amp_publisher:main',
            'camera_publish = play_face.camera_publish:main',
            'move_subs = play_face.move_subs:main',
            'tasks_subs = play_face.tasks_subs:main',
            'websocket_control_node = play_face.websocket_control:main',
            'setup_pub_node = play_face.setup_pub:main',

        ],
    },
)
