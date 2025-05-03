import rclpy,os

from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String, Bool
from amr_websocket.controls import ControlPublisher,Controls
import json
import time

from arduino_msgs.msg import Buttons, Stations
from sensor_msgs.msg import Joy
from control_panel.controlers.system_subs import JoyStick
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
import subprocess
from ament_index_python import get_package_share_directory
import tf_transformations
from tf2_ros import TransformListener, Buffer

MAP_PATH = os.path.join(get_package_share_directory('rviz2py'),'maps')
KEEPOUT_PATH = os.path.join(get_package_share_directory('rviz2py'),'maps','keepout')
CONFIG_RVIZ = os.path.join(get_package_share_directory('rviz2py'),'config')
CONFIG = os.path.join(get_package_share_directory('control_panel'),'config')

class DeliveryControl(Node):
    def __init__(self):
        super().__init__('delivery_control')
        self.get_logger().info("Delivery Control initialized ...")
        self.buttons = Buttons()
        self.stations = Stations()
        self.controler_pub = ControlPublisher('delivery')
        self.controls = Controls()
        self.delivered = False
        self.delivery_completed = False
        self.save = True
        self.location = False
        self.map_save = True
        self.led_status = True
        self.number = 0
        self.crnt_delivery_index = 0
        self.json_file_path = os.path.join(CONFIG, 'mapping_poses.json')
        self.mode_json = os.path.join(CONFIG, 'modes.json')
        self.wifi_status_json = os.path.join(CONFIG, 'wifi_status.json')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.arduino_sub = self.create_subscription(Buttons, 'arduino_topic', self.arduino_callback, 10)
        self.station_sub = self.create_subscription(Stations, 'task_stations', self.station_callback, 10)
        self.station_sub = self.create_subscription(Int32, 'goal_to_pose_feedback', self.feedback_callback, 10)
        self.reboot_sub = self.create_subscription(String, 'button_status', self.reboot_callback, 10)
        self.map_sub = self.create_subscription(String, 'map_save', self.map_callback, 10)
        self.remove_map_sub = self.create_subscription(String, 'map_remove', self.map_remove_callback, 10)
        self.map_default_sub = self.create_subscription(String, 'default_map', self.map_default_callback, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.color_pub = self.create_publisher(String, 'arduino_color', 10)
        self.delivery_publisher = self.create_publisher(Int32, 'delivery_feedback', 10)
        self.charging_pub = self.create_publisher(Bool, '/auto_dock', 10)
        self.timer = self.create_timer(0.01, self.check_goal_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mode_status = self.controls.read_pose('modes')
        self.mode_name = self.mode_status['mode']
        if self.mode_name == 'mapping':
            self.controls.clear_map_pose()
        
    def get_tf_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = trans.transform.translation
            orientation = trans.transform.rotation

            return {
                'position': {
                    'x': position.x,
                    'y': position.y,
                    'z': position.z,
                },
                'orientation': {
                    'x': orientation.x,
                    'y': orientation.y,
                    'z': orientation.z,
                    'w': orientation.w,
                }
            }
        except Exception as e:
            pass
            # self.get_logger().error(f'TF verisi alınamadı: {e}')
            return None
    
    def odom_callback(self, msg):
        if self.led_status:
            self.get_logger().info('The led is opened')
            self.controler_pub.publish_color('o\n')
            time.sleep(1)
            self.controler_pub.publish_color('m\n')
            time.sleep(1)
            self.controler_pub.publish_color('n\n')
            self.led_status = False
        
        self.robot_position = self.get_tf_position()

        if self.location:
            self.location = False
            self.controls.save_to_json(self.robot_position,self.json_file_path,self.station_name)
            self.get_logger().info('Kaydedildi')
    
    def arduino_callback(self, btn):
        if btn.start_btn == 1:
            self.process_delivery()
        if btn.hotspot_btn == 1:
            self.controls.save_json("wifi_mode","hotspot",self.mode_json)
            self.controls.save_json("mode","mapping",self.mode_json)
            self.controls.save_json("ip","10.42.0.1",self.wifi_status_json)
            self.controler_pub.publish_color('m\n')
            time.sleep(1)
            self.controler_pub.publish_color('w\n')
            time.sleep(2)
            self.get_logger().info('Reboot')
            subprocess.call(f'reboot', shell=True)

        
    def station_callback(self, stn):
        self.stations = stn
    def joy_callback(self,msg):
        self.buttons = msg.buttons
        # self.get_logger().info(f'joy callback {self.buttons[3]}')
        if self.buttons[3] == 1:
            #save_map
            if self.map_save:
                self.map_save = False
                self.joy_name = 'joy_map'
                subprocess.Popen(
                        f'cd ~ && cd {MAP_PATH} && ros2 run nav2_map_server map_saver_cli -f {self.joy_name} && cp {MAP_PATH}/{self.joy_name}.yaml {KEEPOUT_PATH} &&  cp {MAP_PATH}/{self.joy_name}.pgm {KEEPOUT_PATH}',
                        stdout=subprocess.PIPE,
                        shell=True,
                        preexec_fn=os.setsid
                )
                self.controler_pub.publish_color('n\n')
                time.sleep(1)
                self.controler_pub.publish_color('m\n')

        if self.buttons[0] == 1 or self.buttons[1] == 1:
            self.save = True
            self.map_save = True
            
        if self.buttons[2] == 1:
            self.map_save = True
            if self.save:
                self.get_logger().info(f'joy callback {self.number}')

                self.number += 1
                self.station_name = f'{self.number}.mpstn'
                self.save = False
                self.location = True
                self.controler_pub.publish_color('y\n')
                time.sleep(1)
                self.controler_pub.publish_color('m\n')

        
    def reboot_callback(self,button_status):
        self.btn_status = button_status.data
        self.get_logger().info(f'Sistem')
        if self.btn_status == 'reboot':
            subprocess.call(f'reboot', shell=True)
            self.get_logger().info(f'Sistem reboot atiliyor {self.btn_status}')
            self.controler_pub.publish_color('w\n')
        if self.btn_status == 'power_off':
            subprocess.call(f'shutdown -h now', shell=True)
            self.get_logger().info(f'Sistem poweroff atiliyor {self.btn_status}')
            self.controler_pub.publish_color('w\n')
    def map_callback(self,map_name):
        self.map_name = map_name.data
        self.get_logger().info(f'{self.map_name} isimli harita Kaydedildi')
        subprocess.Popen(
                f'cd ~ && cd {MAP_PATH} && ros2 run nav2_map_server map_saver_cli -f {self.map_name} && cp {MAP_PATH}/{self.map_name}.yaml {KEEPOUT_PATH} &&  cp {MAP_PATH}/{self.map_name}.pgm {KEEPOUT_PATH}',
                stdout=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid
        )
        self.controler_pub.publish_color('g\n')
        time.sleep(1)
        self.controler_pub.publish_color('m\n')
    def map_remove_callback(self,name):
        map_name = name.data
        self.get_logger().info(f'{map_name} SILINDI')
        maps_extensions = ['.pgm','.yaml','.nct']
        keepouts_extensions = ['.pgm','.yaml']
        for ext in maps_extensions:
            map_paths = os.path.join(MAP_PATH, f"{map_name}{ext}")
            if os.path.exists(map_paths):
                os.remove(map_paths)
        for ext in keepouts_extensions:
            keepout_paths = os.path.join(KEEPOUT_PATH, f"{map_name}{ext}")
            if os.path.exists(keepout_paths):
                os.remove(keepout_paths)
                
    def map_default_callback(self,default_name):
        self.default_name = default_name.data
        self.get_logger().info(f'{self.default_name} isimli harita DEFAULT')
        file_path = os.path.join(CONFIG_RVIZ, 'default_map.yaml')
        default_map_data = {
            'default_map': self.default_name,
            'default_keepout': self.default_name,
        }
        with open(file_path, 'w') as file:
            file.write(json.dumps(default_map_data))
        self.controler_pub.publish_color('g\n')
        time.sleep(1)
        self.controler_pub.publish_color('m\n')
    
    def process_delivery(self):
        if not self.delivered and len(self.stations.stations) > 0:
            if self.crnt_delivery_index < len(self.stations.stations):
                station = self.stations.stations[self.crnt_delivery_index]
                self.get_logger().info(f'*****SENDING GOAL TO {station} STATION')
                self.controler_pub.send_goal(station)
                self.delivered = True
            elif self.crnt_delivery_index == len(self.stations.stations):
                self.controler_pub.send_goal('initialpose')
                self.delivered = True
                self.delivery_completed = True
            self.controler_pub.publish_color('i\n')
            self.crnt_delivery_index += 1
        if self.delivered and len(self.stations.stations) > 0:
            if not self.delivery_completed:
                station = self.stations.stations[self.crnt_delivery_index - 1]
                self.controler_pub.send_goal(station)
            else:
                self.controler_pub.send_goal('initialpose')
    
    def check_goal_callback(self):
        if self.delivered:
            self.controler_pub.results()
    
    def feedback_callback(self, msg):
        if msg.data == 1:
            self.delivered = False
            self.controler_pub.publish_color('g\n')
            self.publish_feedback()
            if self.delivery_completed:
                self.crnt_delivery_index = 0
                self.delivery_completed = False
                # self.docking_pub(True)
    
    def publish_feedback(self):
        msg = Int32()
        msg.data = self.crnt_delivery_index
        self.delivery_publisher.publish(msg)
    
    def publish_color(self, color):
        clr = String()
        clr.data = color
        self.color_pub.publish(clr)
    def docking_pub(self, msg):
        dock = Bool()
        dock.data = msg
        self.charging_pub.publish(dock)
    # def docking_sub(self,msg):
    #     self.dock = msg.data
    #     if self.dock:
    #         self.get_logger().info(f'Docking at {self.dock}')
    #         self.controler_pub.send_goal('docker')
    #         self.docking_pub(True)


def main(args=None):
    rclpy.init(args=args)

    delivery_control = DeliveryControl()
    rclpy.spin(delivery_control)

    delivery_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()