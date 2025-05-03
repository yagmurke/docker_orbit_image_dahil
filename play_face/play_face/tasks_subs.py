import rclpy
from rclpy.node import Node
import time
import threading

from orbit_command_msgs.msg import TasksList
from arduino_msgs.msg import Arduino
from orbit_command_msgs.srv import Face, Records
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Orbit_Tasks_Node(Node):

    def __init__(self):
        super().__init__('Orbit_Tasks_Node')
        self.get_logger().info(f'******Orbit_TASK_Node_Active******')

        self.task_index = 0
        self.task_status = True
        self.loop_status = False
        self.total_task_index = 0

        self.motion_done_event = threading.Event()
        self.motions_status = False

        self.subscription = self.create_subscription(
            TasksList,
            'tasks_topic',
            self.tasks_callback,
            10)

        self.arduinostatus_subscription = self.create_subscription(
            Arduino,
            'arduino_topic',
            self.arduino_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.motions_publisher = self.create_publisher(String, '/motions', 1)

        self.faces_client = self.create_client(Face, 'face_task')
        self.records_client = self.create_client(Records, 'record_task')

        self.timer = self.create_timer(0.03, self.move_tasks)  # 10Hz

    def arduino_callback(self, msg: Arduino):
        self.motions_status = msg.motions_status
        if self.motions_status:
            # self.get_logger().info('Motion finished (from Arduino)')
            self.motion_done_event.set()

    def tasks_callback(self, msg: TasksList):
        self.currunt_tasks = msg
        self.task_index = 0
        self.total_task_index = len(self.currunt_tasks.command)
        self.move_tasks()

    def move_tasks(self):
        try:
            if not self.task_status:
                return  # Görev hâlâ çalışıyor, yeni başlatma

            if self.task_index < self.total_task_index and self.currunt_tasks.command:
                command = self.currunt_tasks.command[self.task_index]
                self.get_logger().info(f'Command: {command}')

                if command.message_type == 1:
                    face = command.face
                    self.get_logger().info(f'Face: {face}')
                    self.task_status = False
                    self.request_faces(face)

                elif command.message_type == 2:
                    record = command.record
                    self.get_logger().info(f'Record: {record}')
                    self.task_status = False
                    self.request_records(record)

                elif command.message_type == 3:
                    motion = command.motion
                    self.get_logger().info(f'Motion: {motion}')
                    self.task_status = False
                    self.motion_publish(motion)

                elif command.message_type == 5:
                    self.get_logger().info(f'Move started')
                    moves = command.command
                    # self.get_logger().info(f'Move: {moves}')
                    self.task_status = False
                    for cmd in moves:
                        self.get_logger().info(f'Command: {cmd} , time: {cmd.time}')
                        msg = Twist()
                        msg.linear.x = cmd.linear_x
                        msg.angular.z = cmd.angular_z
                        self.publisher.publish(msg)
                        time.sleep(cmd.time)
                    self.get_logger().info(f'Move Finished')
                    time.sleep(1)
                    self.task_status = True
                    self.main_loop()

        except Exception as e:
            self.get_logger().error(f'Exception in move_tasks: {str(e)}')

    def main_loop(self):
        self.task_status = True
        self.task_index += 1
        self.move_tasks()

        if self.task_index == self.total_task_index and self.loop_status:
            self.get_logger().info(f'All tasks completed, Restarting')
            time.sleep(1)
            self.task_index = 0

    def request_faces(self, data):
        request = Face.Request()
        request.face = data
        future = self.faces_client.call_async(request)
        future.add_done_callback(self.faces_callback)

    def faces_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service Response (Face): {response.response}')
            self.main_loop()
        except Exception as e:
            self.get_logger().error(f'Service call failed (Face): {str(e)}')

    def request_records(self, data):
        request = Records.Request()
        request.records = data
        future = self.records_client.call_async(request)
        future.add_done_callback(self.records_callback)

    def records_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service Response (Records): {response.response}')
            self.main_loop()
        except Exception as e:
            self.get_logger().error(f'Service call failed (Records): {str(e)}')

    def motion_publish(self, data):
        self.motion_done_event.clear()

        msg = String()
        msg.data = data
        self.motions_publisher.publish(msg)
        self.get_logger().info(f'Motion published: {data}')

        # Motion bitişini bekle (maksimum 10 saniye bekle)
        if self.motion_done_event.wait(timeout=10.0):
            self.get_logger().info('Motion completed (event triggered)')
        else:
            self.get_logger().warn('Motion did not complete in time!')

        self.task_status = True
        self.main_loop()


def main(args=None):
    rclpy.init(args=args)
    node = Orbit_Tasks_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
