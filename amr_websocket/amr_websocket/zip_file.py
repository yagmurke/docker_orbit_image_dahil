import shutil
import os 

from ament_index_python import get_package_share_directory

MAP_PATH = os.path.join(get_package_share_directory('rviz2py'), 'maps')

shutil.make_archive("myzip", "zip", MAP_PATH)