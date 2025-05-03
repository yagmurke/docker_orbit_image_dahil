import rclpy
import copy
import numpy as np
from rclpy.node import Node, Publisher
from nav_msgs.msg import OccupancyGrid
from amr_websocket_interfaces.msg import CompressedOccupancyGrid, UpdatedOccupancyGrid

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class DownsampleMap(Node):
    def __init__(self):
        super().__init__('downsample_map')
        self.get_logger().info('DownsampleMap node started')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Depth of 1 to simulate latched topic
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_msg = None
        self.map_publisher = self.create_publisher(CompressedOccupancyGrid, '/map_downsampled', qos_profile)
        self.costmap_publisher = self.create_publisher(CompressedOccupancyGrid, '/costmap_downsampled', qos_profile)

        self.update_publisher = self.create_publisher(UpdatedOccupancyGrid, '/updated_map', qos_profile)
        self.update_costmap_publisher = self.create_publisher(UpdatedOccupancyGrid, '/updated_costmap', qos_profile)

        self.map_compressor = MapCompressor(self.map_publisher, self.update_publisher)
        self.costmap_compressor = MapCompressor(self.costmap_publisher, self.update_costmap_publisher)
    
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos_profile
        )
    def map_callback(self, msg:OccupancyGrid):
        self.map_compressor.publish_updates(msg, self.get_clock().now().to_msg())
    
    def costmap_callback(self, msg:OccupancyGrid):
        self.costmap_compressor.publish_updates(msg, self.get_clock().now().to_msg())

class MapCompressor:
    def __init__(self, map_publisher:Publisher, map_update_publisher:Publisher):
        self.map_publisher = map_publisher
        self.map_update_publisher = map_update_publisher
        self.old_map = None
    
    def run_length_encode(self, map_data: list):
        # Convert input list to a NumPy array
        data = np.array(map_data)
        
        # Find the indices where values change
        diff_indices = np.where(data[1:] != data[:-1])[0] + 1
        
        # Append start and end indices
        indices = np.concatenate(([0], diff_indices, [len(data)]))
        
        # Calculate run lengths and values
        lengths = np.diff(indices)
        values = data[indices[:-1]]
        print(f'Lengths: {lengths.shape}, Values: {values.shape}')
        
        return values.tolist(), lengths.tolist()

    def compress_map(self, map_msg):
        compressed_map = CompressedOccupancyGrid()
        compressed_map.map_grid = copy.deepcopy(map_msg)
        keys, lengths = self.run_length_encode(compressed_map.map_grid.data)
        compressed_map.map_grid.data = keys
        compressed_map.length= lengths
        return compressed_map
    
    def publish_updates(self, new_map:OccupancyGrid, stamp):
        if self.old_map is not None:
            if new_map.info.width != self.old_map.info.width or new_map.info.height != self.old_map.info.height:
                print('Map dimensions are not equal')
                self.map_publisher.publish(self.compress_map(new_map))
                self.old_map = copy.deepcopy(new_map)
                return
            if np.array_equal(new_map.data, self.old_map.data):
                print('Maps are equal')
            else:
                print('Maps are not equal')
                new_data_np = np.array(new_map.data)
                old_data_np = np.array(self.old_map.data)
                changed_indeces = np.where(new_data_np != old_data_np)[0]
                if len(changed_indeces) > 0:
                    print(f'Changed indeces: {(changed_indeces.shape)}, indices_values: {new_data_np[changed_indeces].shape}')
                    updated_map = UpdatedOccupancyGrid()
                    updated_map.header.frame_id = "map"
                    updated_map.header.stamp = stamp
                    updated_map.index = changed_indeces.tolist()
                    updated_map.data = new_data_np[changed_indeces].tolist()
                    self.map_update_publisher.publish(updated_map)
                else:
                    print('Map was not changed')
        else:
            print('Old Map is None')
            self.map_publisher.publish(self.compress_map(new_map))
        self.old_map = copy.deepcopy(new_map)

def main(args=None):
    rclpy.init(args=args)
    node = DownsampleMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
