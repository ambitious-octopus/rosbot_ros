import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    """
    Subscriber for image messages
    """
    def __init__(self, ultralytics_publisher):
        super().__init__('image_subscriber')
        self.ultralytics_publisher = ultralytics_publisher
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        """
        Callback for the image subscriber
        """
        self.ultralytics_publisher.publish(msg)


class UltralyticsPublisher(Node):
    """
    Node for publishing detection and segmentation results from ultralytics
    """
    def __init__(self):
        super().__init__('ultralytics_publisher')
        self.det_img_pub = self.create_publisher(Image, 
                                               'ultralytics/detection/image', 
                                               10)
        self.det_str_pub = self.create_publisher(String, 
                                               'ultralytics/detection/classes', 
                                               10)
        
        self.seg_img_pub = self.create_publisher(Image, 
                                               'ultralytics/segmentation/image', 
                                               10)
        self.seg_str_pub = self.create_publisher(String, 
                                               'ultralytics/segmentation/classes', 
                                               10)
        
        self.cv_bridge = CvBridge()
        self.det_model = YOLO("yolov8m.pt")
        self.seg_model = YOLO("yolov8m-seg.pt")
        
    def build_str_msg(self, result):
        """
        Build a string message from the result of the ultralytics model
        """
        names = result[0].names
        objects = [names[int(idx)] for idx in result[0].boxes.cls.tolist()]
        return String(data=str(objects))
    
    def build_img_msg(self, result):
        """
        Build an image message from the result of the ultralytics model
        """
        ann = result[0].plot(show=False)
        return self.cv_bridge.cv2_to_imgmsg(ann, encoding="passthrough")
        
    def publish(self, detection):
        """
        Publish the detection and segmentation results
        """
        img = self.cv_bridge.imgmsg_to_cv2(detection)
        seg_result, det_result = self.seg_model(img), self.det_model(img)
        self.det_img_pub.publish(self.build_img_msg(det_result))
        self.det_str_pub.publish(self.build_str_msg(det_result))
        self.seg_img_pub.publish(self.build_img_msg(seg_result))
        self.seg_str_pub.publish(self.build_str_msg(seg_result))


if __name__ == '__main__':
    rclpy.init()
    detection_publisher = UltralyticsPublisher()
    minimal_subscriber = ImageSubscriber(detection_publisher)

    rclpy.spin(minimal_subscriber)