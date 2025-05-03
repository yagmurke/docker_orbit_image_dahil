import os
import subprocess
import signal
import json
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory

from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped, Twist

# from elteksmak_demo.robot_navigator import BasicNavigator, NavigationResult
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
WORKSPACE = '/home/ovali/new_ws'
MAP_PATH = f'{WORKSPACE}/src/hub_robot/hub_motor/maps/test'
station_path = f'{WORKSPACE}/src/hub_robot/elteksmak_demo'
CONFIG = os.path.join(get_package_share_directory('control_panel'),'config')

class Controls:
    def __init__(self):
        self.map_init = None
        self.sudo_password = 'nct.2525'

    def mapping_init(self):
        map_init = subprocess.Popen(
                    f'ros2 launch hub_motor online_sync_launch.py',
                    stdout=subprocess.PIPE,
                    shell=True,
                    preexec_fn=os.setsid
                )
        return map_init
    
    def save_map(self):
        save_map = subprocess.Popen(
                    f'cd ~ && cd {MAP_PATH} && ros2 run nav2_map_server map_saver_cli -f fabrika && cp ./fabrika.yaml ./keepout',
                    stdout=subprocess.PIPE,
                    shell=True,
                    preexec_fn=os.setsid
                )
    
    def kill_process(self, process_id):
        os.killpg(os.getpgid(process_id.pid), signal.SIGINT)
    
    def kill_pid(self, pid):
        os.killpg(pid, signal.SIGINT)
    
    def shutdown(self):
        subprocess.call(f'shutdown -h now', shell=True)
    
    
    def laser(self):
        update_process = subprocess.Popen(
            f'ros2 launch rplidar_ros rplidar_s2_launch.py', 
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid
        )
        return update_process
    def update_system(self):
        update_process = subprocess.Popen(
            f'cd ~ && cd {WORKSPACE} && colcon build', 
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid
        )
    
    def init_multicast(self):
        init_multi = subprocess.call(
            f'echo {self.sudo_password} | sudo -S ifconfig lo multicast', 
            shell=True
        )
    
    def launch_system(self):
        system_launch = subprocess.Popen(
            'ros2 launch hub_motor system.launch.xml',
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid
        )

        nav2 = subprocess.Popen(
            'ros2 launch hub_motor nav2.launch.py',
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid
        )


        return [system_launch, nav2]
    
    def save_pose(self, station_name, pos, file_name):
        data = self.read_pose(file_name)
        data = self.set_data(data, station_name, pos)
        self.save_jsn(data, file_name)
    
    def save_to_json(self, data,path,station_name):
        with open(path, 'r') as file:
            self.data = json.load(file)

        key = station_name
        self.data[key] = data

        with open(path, 'w') as json_file:
            json.dump(self.data, json_file, indent=4)
    
    def set_data(self, data, station_name, pos):
        pose = pos
        data[station_name] = {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation": {
                "w": pose.orientation.w,
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z
            }
        }
        return data
    
    def save_jsn(self, data, file_name):
        with open(os.path.join(CONFIG, f'{file_name}.json'), "w") as outfile:
            json.dump(data, outfile, indent=4)

    
    def read_pose(self, file_name):
        f = open(os.path.join(CONFIG, f'{file_name}.json'))
        data = json.load(f)
        f.close
        return data
    
    def read_station(self):
        f = open(os.path.join(CONFIG, 'stations.json'))
        data = json.load(f)
        f.close
        return data
    
    def save_stations(self, stations):
        with open(os.path.join(CONFIG, 'stations.json'), 'w') as outfile:
            json.dump(stations, outfile, indent=4)
    
    def clear_pose(self):
        data = {}
        with open(os.path.join(CONFIG, 'station_poses.json'), "w") as outfile:
            json.dump(data, outfile, indent=4)
            
    def clear_map_pose(self):
        data = {}
        with open(os.path.join(CONFIG, 'mapping_poses.json'), "w") as outfile:
            json.dump(data, outfile, indent=4)
    def save_json(self,data_name,data,file_path):
        set_data = {f'{data_name}': data}
        try:
            if os.path.exists(file_path):
                with open(file_path,"r") as infile:
                    previous_data= json.load(infile)
            else:
                previous_data = {}
            previous_data.update(set_data)
            with open(file_path,'w') as outfile:
                json.dump(previous_data,outfile, indent=4)
        except Exception as e:
            print(e)

class ControlPublisher(Node):
    def __init__(self, name='control_publisher'):
        super().__init__(name)
        self.get_logger().info('**** Control_Publisher initialized ****')

        self.navigator = BasicNavigator(f'{name}_navigator')

        self.feedback_publisher = self.create_publisher(Int32, 'goal_to_pose_feedback', 10)
        self.color_pub = self.create_publisher(String, 'arduino_color', 10)
        self.cmd_pub=self.create_publisher(Twist,"cmd_vel",10)
        self.charging_pub = self.create_publisher(Bool, '/docking_topic', 10)

        self.pose_msg = PoseWithCovarianceStamped()
        self.initalpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        self.vel=Twist()
    
    def publish_initialpose(self, initial_name, pose_type):
        if pose_type == 'mapping':
            self.stations = Controls().read_pose('mapping_poses')
        elif pose_type == 'navigation':
            self.stations = Controls().read_pose('station_poses')
        if initial_name in self.stations:
            initialpose = self.stations[initial_name]
            self.pose_msg.header.frame_id = 'map'
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_msg.pose.pose.position.x = initialpose['position']['x']
            self.pose_msg.pose.pose.position.y = initialpose['position']['y']
            self.pose_msg.pose.pose.position.z = initialpose['position']['z']
            self.pose_msg.pose.pose.orientation.x = initialpose['orientation']['x']
            self.pose_msg.pose.pose.orientation.y = initialpose['orientation']['y']
            self.pose_msg.pose.pose.orientation.z = initialpose['orientation']['z']
            self.pose_msg.pose.pose.orientation.w = initialpose['orientation']['w']
            self.initalpose_pub.publish(self.pose_msg)
        else:
            self.get_logger().info(f'{self.stations}')
    
    def send_goal(self, station_name):
        goal_pose = PoseStamped()
        pose_dict = Controls().read_pose('station_poses')
        self.get_logger().info(f'station_name={station_name}, {pose_dict}')
        if station_name in pose_dict:
            pose = pose_dict[station_name]
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose["position"]["x"]
            goal_pose.pose.position.y = pose["position"]["y"]
            goal_pose.pose.position.z = pose["position"]["z"]
            goal_pose.pose.orientation.w = pose["orientation"]["w"]
            goal_pose.pose.orientation.x = pose["orientation"]["x"]
            goal_pose.pose.orientation.y = pose["orientation"]["y"]
            goal_pose.pose.orientation.z = pose["orientation"]["z"]
            succ = self.navigator.goToPose(goal_pose)
            self.get_logger().info(f'{succ}')
            return succ
    
    def publish_color(self, color):
        clr = String()
        clr.data = color
        self.color_pub.publish(clr)
    
    def send_vel(self, x, z):
        self.vel.linear.x=x
        self.vel.linear.y=0.0
        self.vel.linear.z=0.0
        self.vel.angular.z = z
        self.cmd_pub.publish(self.vel)
    
    def results(self):
        msg = Int32()
        if self.navigator.isTaskComplete():
            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                msg.data = 1
                self.feedback_publisher.publish(msg)
        else:
            msg.data = 0
            self.feedback_publisher.publish(msg)
    
    def docking_pub(self, msg):
        dock = Bool()
        dock.data = msg
        self.charging_pub.publish(dock)