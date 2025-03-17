import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.last_save_time = time.time()
        self.save_interval = 1.0
        self.image_count = 0
        self.output_dir = 'captured_images'

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        self.get_logger().info('Image saver node started, saving images to "%s"' % self.output_dir)

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(str(e))
                return

            filename = os.path.join(self.output_dir, f'image_{self.image_count:04d}.jpg')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image to {filename}')

            self.image_count += 1
            self.last_save_time = current_time


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()