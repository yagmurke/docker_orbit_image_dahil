import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
import time
import os
import cv2

class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        self.bridge = CvBridge()
        self.img_base64 = None
        self.frame = None
        self.new_frame_received = False

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', self.frame)
            self.img_base64 = base64.b64encode(buffer).decode('utf-8')
            self.new_frame_received = True
            print("✅ Görüntü başarıyla alındı ve base64'e dönüştürüldü!")
        except Exception as e:
            print(f"❌ Görüntü işlenemedi: {e}")
            self.new_frame_received = False

# Global olarak ROS node ve init durumu takip ediliyor
camera_node = None

def init_camera_node():
    global camera_node
    if not rclpy.ok():
        rclpy.init()
    if camera_node is None:
        camera_node = CameraListener()

def capture_image_from_ros(timeout_sec=5.0):
    init_camera_node()
    global camera_node

    # Yeni bir görüntü bekliyoruz
    camera_node.new_frame_received = False
    waited = 0.0
    interval = 0.1
    while not camera_node.new_frame_received and waited < timeout_sec:
        rclpy.spin_once(camera_node, timeout_sec=interval)
        waited += interval

    if camera_node.img_base64:
        return camera_node.img_base64, camera_node.frame
    else:
        print("❌ Kamera hatası: Görüntü alınamadı")
        return None, None

def test_camera():
    print("📷 Kamera test ediliyor...")
    img_base64, frame = capture_image_from_ros()

    if img_base64:
        test_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_test")
        os.makedirs(test_dir, exist_ok=True)
        test_path = os.path.join(test_dir, f"test_camera_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
        cv2.imwrite(test_path, frame)
        print(f"✅ Test görüntüsü kaydedildi: {test_path}")
        return True
    else:
        return False

# Test amacıyla doğrudan çalıştırıldığında
if __name__ == "__main__":
    if test_camera():
        print("✅ Kamera testi başarılı!")
    else:
        print("❌ Kamera testi başarısız!")
