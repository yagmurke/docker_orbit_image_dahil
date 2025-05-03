import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path

from PyQt5.QtCore import QThread, pyqtSignal

class MapListener(QThread):
    map_data = pyqtSignal(OccupancyGrid)
    costmap_data = pyqtSignal(OccupancyGrid)
    tf_message = pyqtSignal(PoseStamped)
    laser_scan = pyqtSignal(LaserScan)
    path_msg = pyqtSignal(Path)

    def __init__(self):
        super(MapListener, self).__init__()
        # rclpy.init(args=None)
        self.node = Node('map_listener')
        # Define a custom QoS profile
        self.cost_data = None
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Depth of 1 to simulate latched topic
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.node.get_logger().info("Waiting for /map topic...")
        self.subscription = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )
        self.global_costmap = self.node.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos_profile
        )

        self.lidar_sub = self.node.create_subscription(
            LaserScan,
            "/scan_multi",
            self.laser_callback,
            10
        )

        self.path_sub = self.node.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )

        self.cost_map_update = self.node.create_timer(5.0, self.update_cost)


        self.node.get_logger().info("/map topic is now available and subscribed.")

        self.timer = self.node.create_timer(0.1, self.on_timer)
        self.buffer = Buffer()
        self.tf_lister = TransformListener(self.buffer, self.node)

    def run(self):
        excutor = MultiThreadedExecutor()
        excutor.add_node(self.node)
        while rclpy.ok():
            try:
                excutor.spin_once()
            except Exception as e:
                self.node.get_logger().info(f"Can't spin {e}")
    def map_callback(self, data):
        self.node.get_logger().info("Received map data")
        self.map_data.emit(data)
    
    def costmap_callback(self, data:OccupancyGrid):
        self.cost_data = data
    
    def laser_callback(self, data:LaserScan):
        self.laser_scan.emit(data)
    
    def path_callback(self, path_data:Path):
        self.path_msg.emit(path_data)
    
    def update_cost(self):
        if self.cost_data is not None:
            self.node.get_logger().info("Cost published")
            self.costmap_data.emit(self.cost_data)
    
    def on_timer(self):
        pos = PoseStamped()
        try:
            t = self.buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            pos.header.frame_id = 'map'
            pos.header.stamp = self.node.get_clock().now().to_msg()
            pos.pose.position.x = t.transform.translation.x
            pos.pose.position.y = t.transform.translation.y
            pos.pose.position.z = 0.0
            pos.pose.orientation.w = t.transform.rotation.w
            pos.pose.orientation.x = t.transform.rotation.x
            pos.pose.orientation.y = t.transform.rotation.y
            pos.pose.orientation.z = t.transform.rotation.z
            self.tf_message.emit(pos)
        except TransformException as ex:
            pass
            # self.node.get_logger().info(ex)



class TfListener(QThread):
    tf_message = pyqtSignal(PoseStamped)
    def __init__(self):
        super(TfListener, self).__init__()

    
    def run(self):
        # rclpy.init(args=None)
        self.node = Node('rviz_tf_listener')
        self.timer = self.node.create_timer(1.0, self.on_timer)
        self.buffer = Buffer()
        self.tf_lister = TransformListener(self.buffer, self.node)
        rclpy.spin(self.node)

    
    def on_timer(self):
        pos = PoseStamped()
        try:
            t = self.buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            pos.header.frame_id = 'map'
            pos.header.stamp = self.get_clock().now().to_msg()
            pos.pose.position.x = t.transform.translation.x
            pos.pose.position.y = t.transform.translation.y
            pos.pose.position.z = 0.0
            pos.pose.orientation.w = t.transform.rotation.w
            pos.pose.orientation.x = t.transform.rotation.x
            pos.pose.orientation.y = t.transform.rotation.y
            pos.pose.orientation.z = t.transform.rotation.z
            self.tf_message.emit(pos)
        except TransformException as ex:
            # pass
            self.get_logger().info(ex)

def main(args=None):
    rclpy.init(args=args)
    map_listener = MapListener()
    # rclpy.spin(map_listener)
    # map_listener.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
