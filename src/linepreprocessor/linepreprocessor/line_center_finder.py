import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()
import cv2
import os
import skimage as ski
from skimage.color import rgb2gray
from skimage import feature
import math

class RawImageSubscriber(Node):
    def __init__(self):
        super().__init__('raw_image_subscriber')
        self.declare_parameter('capture_image', False)
        capture_image_val = self.get_parameter('capture_image').get_parameter_value().bool_value
        self.get_logger().info(f"Capture Image Set To:{capture_image_val}")

        default_capture_dir = os.path.join(os.getcwd(), 'raw_image_capture')
        self.declare_parameter('capture_dir', default_capture_dir)
        capture_dir_val = self.get_parameter('capture_dir').get_parameter_value().string_value
        self.get_logger().info(f"Capture Image Set To:{capture_dir_val}")

        default_crop_x = 200
        self.declare_parameter('crop_x', default_crop_x)
        self.crop_x_val = self.get_parameter('crop_x').get_parameter_value().integer_value
        self.get_logger().info(f"Crop X Set To:{self.crop_x_val}")

        default_crop_y = 200
        self.declare_parameter('crop_y', default_crop_y)
        self.crop_y_val = self.get_parameter('crop_y').get_parameter_value().integer_value
        self.get_logger().info(f"Crop Y Set To:{self.crop_y_val}")
        
        if capture_image_val:
            os.makedirs(capture_dir_val, exist_ok=True)
            
        self.subscription = self.create_subscription(Image, 'image_raw', self.preproc_image_callback, 10)
        self.latest_image = None
        self.subscription

    #saves each incoming image based on ros arg params
    def check_save_img(self, cv_image):
        if self.get_parameter('capture_image').get_parameter_value().bool_value:
            #save_img = cv_image * 255.0
            #save_img = save_img.astype(np.uint8)

            filename = os.path.join(
            self.get_parameter('capture_dir').get_parameter_value().string_value,
            f"frame_{rclpy.clock.Clock().now().nanoseconds}.png")

            cv2.imwrite(filename, cv_image)

            self.get_logger().info(f"Saved Incoming Image to {filename}")

    #convert the image message to cv2 image and flip colors (black to white)
    def preproc_image_callback(self, msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.check_save_img(bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))

            processed_image = self.ImagePreProcess(cv_image, (self.crop_x_val, self.crop_y_val), cv_image.shape[0]/2, cv_image.shape[1]/2)
            self.latest_image = processed_image
            moments = cv2.moments(processed_image.astype(np.uint8), binaryImage=True)
            self.last_center_point = (self.FirstOrderCentroid(moments))
            self.last_radian_dir = self.SecondOrderTilt(moments)

            overlay_img = (processed_image * 255).astype(np.uint8)
            overlay_img = cv2.cvtColor(overlay_img, cv2.COLOR_GRAY2BGR)

            # Draw centroid
            centerX, centerY = int(self.last_center_point[0]), int(self.last_center_point[1])
            cv2.circle(overlay_img, (centerX, centerY), 6, (0, 0, 255), -1)  # red

            # Draw orientation arrow
            length = 50
            end_x = int(centerX + length * math.cos(self.last_radian_dir))
            end_y = int(centerY + length * math.sin(self.last_radian_dir))
            cv2.arrowedLine(overlay_img, (centerX, centerY), (end_x, end_y), (255, 0, 0), 2)  # blue

            self.latest_image = overlay_img


        except Exception as e:
            self.get_logger().error(f"Image Conversion Failed: {e}")

    #Apply basic filtering / thresholding to segment out line from environment
    def ImagePreProcess(self, imageArr, cropSize, centerH, centerW, gausStd = 10, gausTrunc = 1.5, binThresh = 0.99):
        grayscale = rgb2gray(imageArr)
        blur = ski.filters.gaussian(grayscale, gausStd, truncate=gausTrunc)
        blur = ski.util.invert(blur)
        binary = blur > binThresh
        binary_crop = binary[int(centerH - cropSize[0]) : int(centerH + self.crop_x_val), 
                             int(centerW - cropSize[1]) : int(centerW + self.crop_y_val)]
        return binary_crop

    @staticmethod
    #Calculate the center of the line using the center of inertia of binary pixel values
    def CenterMoment(arr, i, j, centroid_x=0, centroid_y=0):
        sum = 0
        for y in range(arr.shape[0]):
            for x in range(arr.shape[1]):
                sum += arr[x, y] * math.pow(x - centroid_x, j) * math.pow(y - centroid_y, i)
        return sum

    @staticmethod
    #Get the central moments (center of line)
    def FirstOrderCentroid(moments):

        # empty mask guard
        if moments["m00"] == 0:           
            raise ValueError("Mask has no foreground pixels")
            return 0
        
        x_hat = moments["m10"] / moments["m00"]
        y_hat = moments["m01"] / moments["m00"]

        #print(f"x pos {x_hat} y pos {y_hat}")
        return x_hat, y_hat

    @staticmethod
    #Get the tilt of the line 
    def SecondOrderTilt(moments):

        mu20 = moments["mu20"] / moments["m00"]
        mu02 = moments["mu02"] / moments["m00"]
        mu11 = moments["mu11"] / moments["m00"]

        theta_rad = 0.5 * math.atan2(2 * mu11, mu20 - mu02)
        #theta_deg = np.degrees(theta_rad) % 180 
        #print(f"{theta_rad} theta degrees: {theta_deg}")

        return theta_rad



class ImageCenterPublisher(Node):

    #constructor requires instance of the subscriber who has pre-processed the image
    def __init__(self, image_sub: RawImageSubscriber):
        super().__init__('image_center_publisher')
        #self.publishers = [self.create_publisher(TFMessage, 'lineCenterTopic', 10),
        #                    self.create_publisher(Image, 'preProcessedImageTopic', 10)]
        self.image_subscriber = image_sub
        self.publisher = self.create_publisher(Image, 'preProcessedImageTopic', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.image_preproc_callback)

    #convert back to image message format for publishing
    def image_preproc_callback(self):
        if self.image_subscriber.latest_image is not None:
            try:
                #image_msg = bridge.cv2_to_imgmsg((self.image_subscriber.latest_image * 255).astype(np.uint8), encoding='mono8')
                image_msg = bridge.cv2_to_imgmsg(self.image_subscriber.latest_image, encoding='bgr8')
                now = self.get_clock().now().to_msg()
                
                image_msg.header.stamp = now
                image_msg.header.frame_id = "camera_link"

                trans = TransformStamped()
                trans.header.stamp = now
                trans.header.frame_id = "base_link"
                trans.child_frame_id = "line_center"
                trans.transform.translation.x = self.image_subscriber.last_center_point[0] * 0.001
                trans.transform.translation.y = self.image_subscriber.last_center_point[1] * 0.001
                qx, qy, qz, qw = quaternion_from_euler(0, 0, np.degrees(self.image_subscriber.last_radian_dir) % 180 )

                self.get_logger().info(f"Line Center Pos:{self.image_subscriber.last_center_point[0], self.image_subscriber.last_center_point[1]}")
                self.get_logger().info(f"Line Orientation Deg:{np.degrees(self.image_subscriber.last_radian_dir)% 180 }")

                trans.transform.rotation.x = qx
                trans.transform.rotation.y = qy
                trans.transform.rotation.z = qz
                trans.transform.rotation.w = qw

                self.publisher.publish(image_msg)
                self.tf_broadcaster.sendTransform(trans)

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
