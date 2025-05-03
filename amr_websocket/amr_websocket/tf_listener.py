import rclpy

from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TfListener(Node):
    def __init__(self):
        super().__init__("websocket_tf")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_frame", "map"),
                ("laser1_frame", "laser"),
                ("laser2_frame", "lidar_2"),
                ("base_link_frame", "base_link")
            ]
        )

        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.laser1_frame = self.get_parameter("laser1_frame").get_parameter_value().string_value
        self.laser2_frame = self.get_parameter("laser2_frame").get_parameter_value().string_value
        self.base_link_frame = self.get_parameter("base_link_frame").get_parameter_value().string_value

        self.tf_publisher = self.create_publisher(TransformStamped, "tf_websocket", 10)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.1, self.tf_timer)
    
    def tf_timer(self):
        frame_pairs = [
            (self.base_link_frame, self.map_frame),
            (self.laser1_frame, self.map_frame),
            (self.laser2_frame, self.map_frame)
        ]

        for target_frame, fixed_frame in frame_pairs:
            try:
                t = self.buffer.lookup_transform(
                    fixed_frame,
                    target_frame,
                    rclpy.time.Time()
                )
                self.tf_publisher.publish(t)
                print(t)
            except TransformException as e:
                print(e)
                self.get_logger().info(f"Can't transform from {target_frame} to {fixed_frame}")

def main():
    rclpy.init(args=None)
    tf_listener = TfListener()
    try:
        rclpy.spin(tf_listener)
    except KeyboardInterrupt:
        tf_listener.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()