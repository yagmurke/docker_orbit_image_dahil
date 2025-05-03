import math 

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

class MapElementTf:
    def m_to_pixel(self, tf:TransformStamped, map_msg:OccupancyGrid):
        tf_pose = [0.0, 0.0, 0.0]
        pose_x = tf.transform.translation.x
        pose_y = tf.transform.translation.y

        map_height = map_msg.info.height
        map_origin_x = map_msg.info.origin.position.x
        map_origin_y = map_msg.info.origin.position.y

        r = R.from_quat([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.w,
            tf.transform.rotation.z
        ])
        yaw = r.as_euler('zyx', degrees=False)

        if map_height > 0:
            map_resolution = map_msg.info.resolution
            map_originx = map_msg.info.origin.position.x
            map_originy = map_msg.info.origin.position.y
            tf_pose = [(pose_x - map_originx) / map_resolution,
                       ((map_originy - pose_y) / map_resolution) + (map_height),
                       yaw[0]]
        return tf_pose
    
    def pixel_to_m(self, pose:list, map_msg:OccupancyGrid):
        map_resolution = map_msg.info.resolution
        map_origin_x = map_msg.info.origin.position.x
        map_origin_y = map_msg.info.origin.position.y
        map_height = map_msg.info.height
        x = pose[0] * map_resolution + map_origin_x
        y = (map_height - pose[1]) * map_resolution + map_origin_y
        q = self.quat(pose[2])

        return [x, y, 0.0], q
    
    def yaw_difference(self, y, x):
        angle_radians = math.atan2(y, x)
        if angle_radians < 0:
            angle_radians += 2 * math.pi
        
        return angle_radians
    
    def quat(self, angle:float):
        q = R.from_euler('zyx', [angle - math.pi, 0, 0], degrees=False)
        orientation = q.as_quat().tolist()
        return orientation