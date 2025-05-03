import os
import socket
import shutil
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ament_index_python import get_package_share_directory

MAP_PATH = os.path.join(get_package_share_directory('rviz2py'), 'maps')
CONFIG = os.path.join(get_package_share_directory('control_panel'),'config')
print(MAP_PATH)
TEMP_ = os.path.join(get_package_share_directory('amr_websocket'), 'resource', 'temp')

class FileSender(Node):
    def __init__(self):
        super().__init__("file_sender")
        self.get_logger().info("File Sender initialized ...")
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(('0.0.0.0', 10000))
        
        # self.create_subscription(Bool, 'websocket_file_client', self.send_file_callback, 10)
        msg = Bool()
        msg.data = True
        self.send_file_callback(msg)
    
    def __create_client(self):
        self.s.listen()
        client, addr = self.s.accept()
        print("listening ...")
        return client
    
    def send_file_callback(self, msg:Bool):
        if msg.data:
            self.get_logger().info("Creating server ...")
            self.client = self.__create_client()
            self.get_logger().info("Client connected.")
            req = self.client.recv(1).decode()
            self.get_logger().info(f'Request: {req}')
            if req == 'r':
                dir_req = self.client.recv(1).decode()
                self.__send_folder(CONFIG) if dir_req == 'c' else self.__send_folder(MAP_PATH)
            elif req == 's':
                dir_req = self.client.recv(1).decode()
                self.__receive_folder(CONFIG) if dir_req == 'c' else self.__receive_folder(MAP_PATH)
            self.client.close()
            self.get_logger().info("Closed socket")
            msg = Bool()
            msg.data = True
            self.send_file_callback(msg)
    
    def __send_folder(self, path):
        shutil.make_archive(os.path.join(TEMP_, 'maps'), 'zip', path)
        temp_map = os.path.join(TEMP_, 'maps.zip')
        self.get_logger().info("Zipped successfully")

        self.client.sendall(struct.pack('!H', len('maps.zip'))) #(2 bytes)
        self.client.send('maps.zip'.encode())

        file_size = os.path.getsize(temp_map)
        self.client.sendall(struct.pack('!I', file_size)) #(4 byte)
        file = open(temp_map, 'rb')
        self.get_logger().info("Sending maps.zip")
        # data = file.read()
        # self.client.sendall(data)
        file_bytes = b""
        while (chunk:=file.read(1024)):
            self.client.sendall(chunk)
            file_bytes += chunk
            perc = int((len(file_bytes)/file_size) * 100)
        file.close()
        os.remove(temp_map)
        self.client.close()
        self.get_logger().info("Sent successfully")
    
    def __receive_folder(self, path):
        raw_name_size = self.client.recv(2)
        name_size = struct.unpack('!H', raw_name_size)[0]
        file_name = self.client.recv(name_size).decode('utf-8')
        self.get_logger().info(f"Receiving {file_name} file")
        raw_file_size = self.client.recv(4)
        file_size = struct.unpack('!I', raw_file_size)[0]
        self.get_logger().info(f"file size: {file_size}")
        file = open(os.path.join(TEMP_, file_name), 'wb')
        file_bytes = b""

        while len(file_bytes) < file_size:
            data = self.client.recv(1024)
            file_bytes += data
        file.write(file_bytes)
        file.close()
        shutil.unpack_archive(os.path.join(TEMP_, file_name), path)
        os.remove(os.path.join(TEMP_, file_name))
        self.client.close()

def main():
    rclpy.init(args=None)
    file_sender_node = FileSender()
    try:
        rclpy.spin(file_sender_node)
    except KeyboardInterrupt:
        file_sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
