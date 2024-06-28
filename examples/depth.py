import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer

class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('multi_topic_listener')
        
        # Create subscribers using message_filters
        self.image_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.cv_bridge = CvBridge()

        # Create a time synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],  # List of subscribers
            10,                      # Queue size
            0.1                      # Slop (seconds)
        )
        
        # Register the callback to the synchronizer
        self.ts.registerCallback(self.callback)

    def callback(self, image, depth):
        image = self.cv_bridge.imgmsg_to_cv2(image)
        depth = self.cv_bridge.imgmsg_to_cv2(depth)
        print(image.shape, depth.shape)

        cv2.imshow('image', np.hstack([image, depth]))
        cv2.waitKey(1)
        cv2.destroyAllWindows()

class UltralyticsPublisher(Node):
    """
    Node for publishing detection and segmentation results from ultralytics
    """
    def __init__(self):
        super().__init__('ultralytics_publisher')
        
        
        self.seg_model = YOLO("yolov8m-seg.pt")
        
    def publish(self, detection):
        """
        Publish the detection and segmentation results
        """
        img = detection
        seg_result, det_result = self.seg_model(img), self.det_model(img)
        self.det_img_pub.publish(self.build_img_msg(det_result))
        self.det_str_pub.publish(self.build_str_msg(det_result))
        self.seg_img_pub.publish(self.build_img_msg(seg_result))
        self.seg_str_pub.publish(self.build_str_msg(seg_result))


if __name__ == '__main__':
    rclpy.init()
    # detection_publisher = UltralyticsPublisher()
    minimal_subscriber = DepthImageSubscriber()

    rclpy.spin(minimal_subscriber)