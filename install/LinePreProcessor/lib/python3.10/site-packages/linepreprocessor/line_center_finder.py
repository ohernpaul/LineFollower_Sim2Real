import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()
import cv2

class RawImageSubscriber(Node):
    def __init__(self):
        super().__init__('raw_image_subscriber')
        self.subscription = self.create_subscription(Image, 'image_raw', self.preproc_image_callback, 10)
        self.latest_image = None
        self.subscription

    #convert the image message to cv2 image and flip colors (black to white)
    def preproc_image_callback(self, msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed_image = cv2.bitwise_not(cv_image)
            self.latest_image = processed_image
        except Exception as e:
            self.get_logger().error(f"Image Conversion Failed: {e}")


class ImageCenterPublisher(Node):

    #constructor requires instance of the subscriber who has pre-processed the image
    def __init__(self, image_sub: RawImageSubscriber):
        super().__init__('image_center_publisher')
        #self.publishers = [self.create_publisher(TFMessage, 'lineCenterTopic', 10),
        #                    self.create_publisher(Image, 'preProcessedImageTopic', 10)]
        self.image_subscriber = image_sub
        self.publisher = self.create_publisher(Image, 'preProcessedImageTopic', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.image_preproc_callback)

    #convert back to image message format for publishing
    def image_preproc_callback(self):
        if self.image_subscriber.latest_image is not None:
            try:
                image_msg = bridge.cv2_to_imgmsg(self.image_subscriber.latest_image, encoding='bgr8')
                self.publisher.publish(image_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to Publish image: {e}')
        else:
            self.get_logger().info('Waiting for image...')

def main(args=None):
    rclpy.init(args=args)

    raw_image_sub = RawImageSubscriber()
    image_center_publisher = ImageCenterPublisher(raw_image_sub)

    #require executer when we want to spin multiple nodes in a single script
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(raw_image_sub)
    executor.add_node(image_center_publisher)
    executor.spin()

    raw_image_sub.destroy_node()
    image_center_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
