import rclpy
from rclpy.node import Node
import time

from orbit_command_msgs.msg import MoveList
from geometry_msgs.msg import Twist

class Orbit_moves_Node(Node):

    def __init__(self):
        super().__init__('Orbit_moves_Node')
        self.get_logger().info(f'******Orbit_TASK_Node_Active******')
        self.task_index = 0
        self.task_status = True
        self.loop_status = False # Loop status degerini bir topicten dinleyerek olustur ve guiden gonder, bu duruma gore loop yapilacak
        self.subscription = self.create_subscription(
            MoveList,
            'movements',
            self.moves_callback,
            1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
    def moves_callback(self, msg:MoveList):
        self.current_moves = msg
        self.get_logger().info(f'{self.current_moves.command}')
        print((self.current_moves.command))
        for cmd in self.current_moves.command:
            self.previus =time.time()
            # self.get_logger().info(f'Received MoveList: time={cmd.time}, linear_x={cmd.linear_x}, angular_z={cmd.angular_z}')
            msg = Twist()
            msg.linear.x = cmd.linear_x
            msg.angular.z = cmd.angular_z
            self.publisher.publish(msg)
            time.sleep(cmd.time)
            
            self.time_loop = time.time() - self.previus
            self.previus = time.time()
            print(self.time_loop)
        print('bitti')

    ##Motions Service
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Orbit_moves_Node()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()