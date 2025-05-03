import rclpy
import time
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

import io
import base64
import numpy as np
import sounddevice as sd
from pydub import AudioSegment

import math
from nav_msgs.msg import Odometry
from orbit_command_msgs.msg import Commands, Status, Stop
from orbit_command_msgs.srv import Records
from std_msgs.msg import Bool


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        
        # Create tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store previous position
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_yaw = 0.0
        self.previous_speed = 0.0
        self.first_transform = True
        self.active_task = False
        self.cmd_sent = False
        self.command_index = 0
        self.speed = 0.12

        self.current_commands = Commands()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.response_pub = self.create_publisher(Status, "orbit_status", 10)
        # Create timer for periodic checking
        # self.timer = self.create_timer(0.03, self.check_transform)  # 10Hz
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.check_transform,
            1
        )
        # Subscribe to odom topic
        self.commands_sub = self.create_subscription(
            Commands,
            'orbit_commands',
            self.command_callback,
            10
        )

        self.stop_sub = self.create_subscription(
            Stop,
            "orbit_stop",
            self.stop_callback,
            10
        )

        self.text_client = self.create_client(
            Records,
            'record_task')
        
        # while not self.text_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')
        # self.get_logger().info('Service available, ready to process requests.')
    
    def send_vel(self, v, w):
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        self.cmd_vel_publisher.publish(vel_msg)
    
    def stop_callback(self, msg:Stop):
        self.send_vel(0.0, 0.0)
        if msg.student_id:
            self.active_task = False
            self.publish_status(msg.student_id, "Stopped")
    
    def command_callback(self, msg: Commands):
        if not self.active_task:
            self.current_commands = msg
            request = Records.Request()
            request.records = f"Haydi {msg.student_name}, programını başlatıyorum. 1,. 2,. 3,. "
            self.get_logger().info(f"Received command: {msg.student_name}")
            self.future = self.text_client.call_async(request)
            self.get_logger().info("Request sent to text-to-speech service")
            self.future.add_done_callback(self.callback_response)

            # self.active_task = True
            # self.publish_status(self.current_commands.student_id, "Running")

    def callback_response(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f"Response: {response.response}")
                self.active_task = True
                self.publish_status(self.current_commands.student_id, "Running")
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error(f'Exception in callback: {str(e)}')
    
    def publish_status(self, student_id, status):
        status_msg = Status()
        status_msg.student_id = student_id
        status_msg.status = status
        self.response_pub.publish(status_msg)

    def check_transform(self, odom:Odometry):
        target_distance = 0.595
        try:
            if self.active_task:

                current_x = odom.pose.pose.position.x
                current_y = odom.pose.pose.position.y
                
                current_yaw = euler_from_quaternion(
                    [
                        odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w
                    ]
                )

                if self.first_transform:
                    self.prev_x = current_x
                    self.prev_y = current_y
                    self.prev_yaw = current_yaw
                    self.first_transform = False
                    return
                
                # Calculate differences
                self.dx = current_x - self.prev_x
                self.dy = current_y - self.prev_y

                # Calculate rotation difference
                dyaw = current_yaw[2] - self.prev_yaw[2]
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2 * math.pi

                # self.get_logger().info(
                #     f'Previous yaw: {self.prev_yaw[2]}, Current yaw: {current_yaw[2]}, Rotation difference - dyaw: {dyaw:.3f} radians'
                # )
                
                # Calculate total distance moved
                distance = math.sqrt(self.dx**2 + self.dy**2)
                
                # self.get_logger().info(
                # print(f'Position difference distance: {distance:.3f}')
                # )
                if self.command_index < len(self.current_commands.commands) and self.current_commands.commands:
                    if self.current_commands.commands[self.command_index].command == 1:
                        move_times = self.current_commands.commands[self.command_index].move
                        if (target_distance * move_times) <= 0.8:
                            self.speed = 0.12
                        else:
                            if (target_distance * move_times) - distance > 0.6:
                                if self.speed < 0.5:
                                    
                                    self.speed += 0.005
                        if (target_distance * move_times) - distance < 0.6:
                            if self.speed > 0.12:
                                self.speed -= 0.005
                        # self.send_vel(0.5, 0.0)
                        if self.speed != self.previous_speed:
                            self.send_vel(self.speed, 0.0)
                        self.previous_speed = self.speed
                        if distance > target_distance * move_times:
                            self.get_logger().info('Target distance reached')
                            self.speed = 0.12
                            self.send_vel(0.0, 0.0)
                            time.sleep(2.0)
                            self.prev_yaw = current_yaw
                            self.prev_x = current_x
                            self.prev_y = current_y 
                            self.cmd_sent = False
                            self.previous_speed = 0.0
                            self.command_index += 1
                    elif self.current_commands.commands[self.command_index].command == 2:
                        direction = self.current_commands.commands[self.command_index].turn
                        if direction == 2:
                            direction = -1
                        if not self.cmd_sent:
                            self.send_vel(0.0, -0.50 * direction)
                            self.cmd_sent = True
                        if abs(dyaw) >= math.pi / 2.0:
                            print(dyaw)
                            self.get_logger().info('Target yaw reached')
                            self.send_vel(0.0, 0.0)
                            time.sleep(2.0)
                            self.prev_yaw = current_yaw
                            self.prev_x = current_x
                            self.prev_y = current_y
                            self.previous_speed = 0.0
                            self.cmd_sent = False
                            self.command_index += 1
                    
                    elif self.current_commands.commands[self.command_index].command == 3:
                        try:
                            command = self.current_commands.commands[self.command_index]
                            audio_bytes = base64.b64decode(command.sound_data)
                           #  sample_rate = command.sample_rate
                            print((len(command.sound_data)))
                            audio = AudioSegment.from_file(io.BytesIO(audio_bytes), format="mp3")
                            #  audio = audio.set_frame_rate(sample_rate).set_channels(1)  # Adjust as needed
                            samples = np.array(audio.get_array_of_samples(), dtype=np.int16)
                            sd.play(samples, samplerate=audio.frame_rate, extra_settings=sd.default.extra_settings)
                            sd.wait()
                            self.get_logger().info("Audio played")
                            self.command_index += 1
                            
                        except Exception as e:
                            self.get_logger().error(f'Error playing audio: {str(e)}')
                    
                    elif self.current_commands.commands[self.command_index].command == 4:
                        self.get_logger().info("Reading temperature value")
                    
                else:
                    self.active_task = False
                    self.first_transform = True
                    self.command_index = 0
                    self.publish_status(self.current_commands.student_id, "Completed")
            
        except TransformException as ex:
            self.get_logger().warning(
                f'Could not transform base_link to odom: {ex}'
            )

def main():
    rclpy.init()
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()