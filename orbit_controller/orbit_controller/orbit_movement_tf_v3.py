import rclpy
import time
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import euler_from_quaternion

import io
import base64
import numpy as np
import sounddevice as sd
from pydub import AudioSegment

import math
from nav_msgs.msg import Odometry
from orbit_command_msgs.msg import Commands, Status, Stop
from orbit_command_msgs.srv import Records
from std_msgs.msg import Bool, Float32


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.get_logger().info('**** PoseMonitor initialized 0.017****')
        
        # Create tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # self.turn_stages = [
        #     (0.0, 0.2),
        #     (0.261, 0.4),
        #     (0.522, 0.6),
        #     (1.044, 0.4),
        #     (1.305, 0.2),
        #     (1.57, 0.0)
        # ]
        # self.turn_stages = [
        #     (0.0, 0.2),
        #     (0.261, 0.35),
        #     (0.522, 0.5),
        #     (1.044, 0.35),
        #     (1.405, 0.15),
        #     (1.57, 0.0)
        # ]
        self.turn_stages = [
            (0.0, 0.2),
            (0.261, 0.35),
            (0.522, 0.5),
            (1.044, 0.35),
            (1.405, 0.15),
            ((math.pi / 2.0)-0.04, 0.0)#0.015
        ]
        self.current_stage = 0
        self.current_move_stage = 0
        
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_yaw = 0.0
        self.first_transform = True
        self.active_task = False
        self.command_index = 0
        self.speed = 0.08

        self.cmd_sent = False
        self.previous_speed = 0.0

        self.current_commands = Commands()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.amp_pub = self.create_publisher(Float32, 'decibel', 10)
        self.response_pub = self.create_publisher(Status, "orbit_status", 10)
        # Create timer for periodic checking
        self.timer = self.create_timer(0.01, self.check_transform)  # 30Hz
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
        
        while not self.text_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service available, ready to process requests.')
    
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

    
    def publish_amp(self, amp):
        decibel = Float32()
        decibel.data = amp
        self.amp_pub.publish(decibel)

    def check_transform(self):
        target_distance = 0.595
        try:
            if self.active_task:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    'odom',
                    'base_link',
                    rclpy.time.Time()
                )
                
                current_x = transform.transform.translation.x
                current_y = transform.transform.translation.y
                
                current_yaw = euler_from_quaternion(
                    [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
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

                # self.get_logger().info(
                #     f'dyaw: {dyaw:.3f} radians'
                # )
                
                # Calculate total distance moved
                distance = math.sqrt(self.dx**2 + self.dy**2)
                
                # self.get_logger().info(
                # print(f'Position difference distance: {distance:.3f}')
                # )
                if self.command_index < len(self.current_commands.commands) and self.current_commands.commands:

                    if self.current_commands.commands[self.command_index].command == 1:
                        move_times = self.current_commands.commands[self.command_index].move
                        total_target_distance = target_distance * move_times

                        # Mesafe eşiklerine göre hızlar belirleniyor
                        self.move_stages = [
                            (0.0, 0.08),
                            (total_target_distance * 0.2, 0.2),
                            (total_target_distance * 0.4, 0.35),
                            (total_target_distance * 0.6, 0.3),  # Reduced from 0.4
                            (total_target_distance * 0.8, 0.2),  # Reduced from 0.25
                            (total_target_distance * 0.9, 0.1),  # Added extra slow stage
                            (total_target_distance, 0.08)        # Reduced final approach speed
                        ]

                        # Şu anki hedef stage
                        target_distance_stage, target_speed = self.move_stages[self.current_move_stage]

                        # Eğer stage mesafesine ulaşıldıysa
                        if distance >= target_distance_stage:
                            self.send_vel(target_speed, 0.0)
                            self.get_logger().info(f"Updated speed to {target_speed} at distance {distance:.2f} m")
                            self.current_move_stage += 1
                            time.sleep(0.1)  # Çok hızlı stage atlamasın diye minik bir bekleme

                        # Eğer tüm stage'ler tamamlandıysa
                        if distance >= total_target_distance:
                            self.get_logger().info('Target distance reached')
                            self.send_vel(0.0, 0.0)
                            time.sleep(2.0)
                            self.prev_yaw = current_yaw
                            self.prev_x = current_x
                            self.prev_y = current_y
                            self.cmd_sent = False
                            self.previous_speed = 0.0
                            self.current_move_stage = 0  # Reset
                            self.command_index += 1

                    elif self.current_commands.commands[self.command_index].command == 2:
                        direction = self.current_commands.commands[self.command_index].turn
                        if direction == 2:
                            direction = -1

                        target_dyaw, target_speed = self.turn_stages[self.current_stage]

                        if abs(dyaw) >= target_dyaw:
                            self.send_vel(0.0, -target_speed * direction)
                            self.get_logger().info(f"Updated speed to {target_speed} at dyaw {abs(dyaw)}")
                            self.current_stage += 1  

                        if self.current_stage >= len(self.turn_stages):
                            self.get_logger().info('Target yaw reached')
                            self.send_vel(0.0, 0.0)
                            time.sleep(2.0)
                            self.prev_yaw = current_yaw
                            self.prev_x = current_x
                            self.prev_y = current_y
                            self.previous_speed = 0.0
                            self.cmd_sent = False
                            self.current_stage = 0 
                            self.current_move_stage = 0
                            self.command_index += 1
                    
                    elif self.current_commands.commands[self.command_index].command == 3:
                        try:
                            command = self.current_commands.commands[self.command_index]
                            audio_bytes = base64.b64decode(command.sound_data)

                            # Ses dosyasını oku
                            audio = AudioSegment.from_file(io.BytesIO(audio_bytes), format="mp3")
                            samples = np.array(audio.get_array_of_samples(), dtype=np.int16)

                            # Eğer stereo ise tek kanala düşür
                            if audio.channels == 2:
                                samples = samples.reshape((-1, 2))
                                samples = samples.mean(axis=1).astype(np.int16)

                            samples = samples.astype(np.float32) / 32768.0  # Normalize [-1.0, 1.0]

                            sr = audio.frame_rate
                            frame_rate = 30  # 30 FPS 
                            frame_samples = sr // frame_rate

                            amplitudes = [
                                np.max(np.abs(samples[i * frame_samples: (i + 1) * frame_samples]))
                                for i in range(len(samples) // frame_samples)
                            ]

                            min_amp, max_amp = min(amplitudes), max(amplitudes)

                            start_time = time.time()

                            # Sesi çal
                            sd.play(samples, samplerate=sr, extra_settings=sd.default.extra_settings)

                            for i, amp in enumerate(amplitudes):
                                lip_height = float(10 + (amp - min_amp) / (max_amp - min_amp) * 110) if max_amp != min_amp else 10.0
                                self.publish_amp(lip_height)
                                # self.get_logger().info(f"Lip Height: {lip_height}")

                                elapsed_time = time.time() - start_time
                                target_time = (i + 1) / frame_rate
                                wait_time = max(0, target_time - elapsed_time)
                                time.sleep(wait_time)
                            sd.wait()  # Ses bitene kadar bekle
                            self.get_logger().info("Audio played")

                            self.command_index += 1

                        except Exception as e:
                            self.get_logger().error(f"play_and_publish_amp hatası: {e}")
                    
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