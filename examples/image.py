import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self, detection_publisher):
        super().__init__('image_subscriber')
        self.detection_publisher = detection_publisher
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.detection_publisher.publish_detection(msg)

class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('ultralytics_publisher')
        self.publisher = self.create_publisher(Image, 
                                               'ultralytics/detection', 
                                               10)
        self.cv_bridge = CvBridge()
        self.yolo = YOLO("yolov8m.pt")
        
    def publish_detection(self, detection):
        img = self.cv_bridge.imgmsg_to_cv2(detection)
        result = self.yolo(img)
        ann = result[0].plot(show=False)
        
        msg = self.cv_bridge.cv2_to_imgmsg(ann, 
                                           encoding="passthrough")
        
        self.publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    detection_publisher = DetectionPublisher()
    minimal_subscriber = ImageSubscriber(detection_publisher)

    rclpy.spin(minimal_subscriber)