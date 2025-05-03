import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            50
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/compressed/image',
            50
        )
        self.get_logger().info('Image Compressor Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            scale_percent = 100  
            width = int(cv_image.shape[1] * scale_percent / 100)
            height = int(cv_image.shape[0] * scale_percent / 100)
            resized_image = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_AREA)

            # resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY) # siyah beyaz yap

            _, compressed_image = cv2.imencode('.jpg', resized_image, [cv2.IMWRITE_JPEG_QUALITY, 20])
            compressed_bytes = np.array(compressed_image).tobytes()

            # CompressedImage mesajı oluştur
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = compressed_bytes

            # Mesajı yayınla
            self.publisher.publish(compressed_msg)
            # self.get_logger().info(f'Compressed frame size: {len(compressed_bytes)} bytes')

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
