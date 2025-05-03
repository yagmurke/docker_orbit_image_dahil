import rclpy
from rclpy.node import Node
from orbit_command_msgs.srv import Records, Face
from arduino_msgs.msg import Arduino

class SetupPublisher(Node):
    def __init__(self):
        super().__init__('setup_publisher')
        self.requests_sent = False
        # Create service clients
        self.record_client = self.create_client(Records, '/record')
        # self.face_client = self.create_client(Face, '/change_face')
        
        # Create subscription to Arduino topic
        self.arduinostatus_subscription = self.create_subscription(
            Arduino,
            'arduino_topic',
            self.arduino_callback,
            10)
        
        # Wait for services to be available
        while not self.record_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Record service not available, waiting...')
        # while not self.face_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Change face service not available, waiting...')
        self.send_setup()
        
    def arduino_callback(self, msg: Arduino):
        self.motions_status = msg.motions_status
        # print(f"Arduino status: {self.motions_status}")
        if self.motions_status and not self.requests_sent:
            self.send_requests()
            self.requests_sent = True

    def send_requests(self):
        # Send empty string to /record service
        record_request = Records.Request()
        record_request.records = "Haydi başlayalım!"
        self.record_client.call_async(record_request)
        
        # # Send 'greeting' to /change_face service
        # face_request = Face.Request()
        # face_request.face = 6
        # self.face_client.call_async(face_request)
    
        self.get_logger().error('GONDERILDIRequests sent to both services')
    def send_setup(self):
        record_request = Records.Request()
        record_request.records = "test"
        self.record_client.call_async(record_request)
def main(args=None):
    rclpy.init(args=args)
    setup_publisher = SetupPublisher()
    rclpy.spin(setup_publisher)  # Changed to rclpy.spin to keep node running
    setup_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from orbit_command_msgs.srv import Records, Face

# #!/usr/bin/env python3


# class SetupPublisher(Node):
#     def __init__(self):
#         super().__init__('setup_publisher')
        
#         # Create service clients
#         self.record_client = self.create_client(Records, '/record')
#         self.face_client = self.create_client(Face, '/change_face')
        
#         # Wait for services to be available
#         while not self.record_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Record service not available, waiting...')
#         while not self.face_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Change face service not available, waiting...')
            
#         self.send_requests()

#     def send_requests(self):
#         # Send empty string to /record service
#         record_request = Records.Request()
#         # record_request.records = "Merhaba, Ben Orbit başlayabiliriz"
#         record_request.records = "Hadi başlayalım!"
#         self.record_client.call_async(record_request)
        
#         # Send 'greeting' to /change_face service
#         face_request = Face.Request()
#         face_request.face = 6
#         self.face_client.call_async(face_request)
        
#         self.get_logger().info('Requests sent to both services')

# def main(args=None):
#     rclpy.init(args=args)
#     setup_publisher = SetupPublisher()
#     rclpy.spin_once(setup_publisher)
#     setup_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()