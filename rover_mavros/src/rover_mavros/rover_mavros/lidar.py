import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Lidar(Node):

    def __init__(self):
        super().__init__('lidar')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)
        self.subscription

    def lidar_callback(self, msg):
        self.get_logger().info(f'[lidar] Min: {min(msg.ranges)}')
        self.get_logger().info(f'[lidar] Max: {max(msg.ranges)}')


def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()