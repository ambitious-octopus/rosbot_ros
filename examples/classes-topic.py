import rospy
import time
from ultralytics import YOLO
import ros_numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
detection_model = YOLO("yolov8x.pt")
rospy.init_node('integration')
time.sleep(1)

classes_pub = rospy.Publisher("/ultralytics/detection/classes", String, queue_size=5)

def callback(data):
    array = ros_numpy.numpify(data)
    if classes_pub.get_num_connections():
        det_result = detection_model(array)
        classes = det_result[0].boxes.cls.cpu().numpy().astype(int)
        names = [det_result[0].names[i] for i in classes]
        classes_pub.publish(String(data=str(names)))
    
rospy.Subscriber("/camera/color/image_raw", Image, callback)

while True:
    rospy.spin()