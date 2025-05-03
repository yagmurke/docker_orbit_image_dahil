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

import numpy as np
import matplotlib.pyplot as plt

class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.get_logger().info('**** PoseMonitor initialized 0.017****')
        
        # Create tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        self.turn_stages = []
        self.current_stage = 0
        self.diff_rotation = 0.0
        
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_yaw = 0.0
        self.first_transform = True
        self.active_task = False
        self.command_index = 0
        self.target_index = 0
        self.target_status = True
        self.target_points = []
        self.speed = 0.1

        self.cmd_sent = False
        self.previous_speed = 0.0

        self.current_commands = Commands()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.amp_pub = self.create_publisher(Float32, 'decibel', 10)
        self.response_pub = self.create_publisher(Status, "orbit_status", 10)
        # Create timer for periodic checking
        self.timer = self.create_timer(0.03, self.check_transform)  # 30Hz
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

                # Calculate total distance moved
                distance = math.sqrt(self.dx**2 + self.dy**2)
                
                if self.command_index < len(self.current_commands.commands) and self.current_commands.commands:
                    if self.target_status:
                        current_x, current_y, current_yaw = self.prev_x, self.prev_y, self.prev_yaw[2]
                        self.target_points = []  # Clear previous target points

                        # Grid cell size
                        cell_size = 0.6

                        for command in self.current_commands.commands:
                            if command.command == 1:  # Move command
                                for _ in range(command.move):
                                    target_x = round(current_x + cell_size * round(math.cos(current_yaw)), 2)
                                    target_y = round(current_y + cell_size * round(math.sin(current_yaw)), 2)
                                    self.target_points.append((target_x, target_y))
                                    current_x, current_y = target_x, target_y
                            elif command.command == 2: 
                                turn = command.turn
                                if turn == 1:
                                    current_yaw -= math.pi / 2
                                elif turn == 2:  
                                    current_yaw += math.pi / 2

                                if current_yaw > math.pi:
                                    current_yaw -= 2 * math.pi
                                elif current_yaw < -math.pi:
                                    current_yaw += 2 * math.pi

                        # Ensure all points are aligned to a grid
                        self.target_points = [(round(x, 2), round(y, 2)) for x, y in self.target_points]

                        # Plot the target points and robot position
                        x_coords, y_coords = zip(*self.target_points) if self.target_points else ([], [])
                        plt.figure(figsize=(8, 8))
                        plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', label='Path')
                        plt.scatter([self.prev_x], [self.prev_y], color='r', label='Robot', zorder=5)
                        plt.grid(True)
                        plt.xticks(np.arange(min(x_coords + (self.prev_x,)) - cell_size, max(x_coords + (self.prev_x,)) + cell_size, cell_size))
                        plt.yticks(np.arange(min(y_coords + (self.prev_y,)) - cell_size, max(y_coords + (self.prev_y,)) + cell_size, cell_size))
                        plt.title("Robotun İzlediği Yol ve Konumu")
                        plt.xlabel("X (metre)")
                        plt.ylabel("Y (metre)")
                        plt.axis('equal')
                        plt.legend()
                        plt.show()

                        self.get_logger().info(f"Generated target points: {self.target_points}")
                        self.target_status = False

                    if self.current_commands.commands[self.command_index].command == 1:
                        print(self.target_index, self.target_points)
                        target_x, target_y = self.target_points[self.target_index]
                        # self.get_logger().info(f"Target coordinates: x={target_x}, y={target_y}")

                        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                        self.get_logger().info(f"Distance to target: {distance_to_target}")
                        # PID controller for alignment
                        kp = 1.0  # Proportional gain
                        ki = 0.0  # Integral gain
                        kd = 0.1  # Derivative gain

                        error = math.atan2(target_y - current_y, target_x - current_x) - current_yaw[2]
                        if error > math.pi:
                            error -= 2 * math.pi
                        elif error < -math.pi:
                            error += 2 * math.pi

                        self.integral = getattr(self, 'integral', 0.0) + error
                        derivative = error - getattr(self, 'previous_error', 0.0)
                        self.previous_error = error

                        angular_correction = kp * error + ki * self.integral + kd * derivative

                        if distance_to_target > 0.6:
                            if self.speed < 0.5:
                                self.speed += 0.00125
                        elif distance_to_target < 0.6:
                            if self.speed > 0.1:
                                self.speed -= 0.0045

                        if self.speed != self.previous_speed or abs(angular_correction) > 0.01:
                            self.send_vel(self.speed, angular_correction)
                        self.previous_speed = self.speed

                        if distance_to_target <= 0.05:  # Close enough to the target
                            self.get_logger().info('Target coordinates reached')
                            self.speed = 0.1
                            self.send_vel(0.0, 0.0)
                            time.sleep(2.0)
                            self.prev_yaw = current_yaw
                            self.prev_x = current_x
                            self.prev_y = current_y
                            self.cmd_sent = False
                            self.previous_speed = 0.0
                            self.current_stage = 0
                            self.command_index += 1
                            self.target_index += 1

                    elif self.current_commands.commands[self.command_index].command == 2:
                        direction = self.current_commands.commands[self.command_index].turn
                        if direction == 2:
                            direction = -1
                            self.diff_rotation = 0.067#67
                            # self.diff_rotation = 0.035
                            self.get_logger().info(f"Turning left {self.diff_rotation}")
                        if direction == 1:
                            direction = 1
                            self.diff_rotation = 0.04
                            # self.diff_rotation = 0.035
                            self.get_logger().info(f"Turning right {self.diff_rotation}")
                            
                        self.turn_stages = [
                            (0.0, 0.3),
                            (0.261, 0.35),
                            (0.522, 0.5),
                            (1.044, 0.35),
                            (1.48, 0.15),
                            ((math.pi / 2.0)-self.diff_rotation, 0.0)#0.015
                        ]

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
                    self.target_index = 0
                    self.target_points = []  # Clear the target points list
                    self.target_status = True
                    
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