import rospy
import time
from ultralytics import YOLO
import ros_numpy
from sensor_msgs.msg import Image
detection_model = YOLO("yolov8x.pt")
segmentation_model = YOLO("yolov8x-seg.pt")
rospy.init_node('integration')
time.sleep(1)

det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)
seg_image_pub = rospy.Publisher("/ultralytics/segmentation/image", Image, queue_size=5)


def callback(data):
    array = ros_numpy.numpify(data)
    if det_image_pub.get_num_connections():
        det_result = detection_model(array)
        det_annotated = det_result[0].plot(show=False)
        det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding='rgb8'))
    
    if seg_image_pub.get_num_connections():
        seg_result = segmentation_model(array)
        seg_annotated = seg_result[0].plot(show=False)
        seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding='rgb8'))
    

rospy.Subscriber("/camera/color/image_raw", Image, callback)

while True:
    rospy.spin()