import rospy
import time
from ultralytics import YOLO
import ros_numpy
from sensor_msgs.msg import Image

model = YOLO("yolov8l.pt")
rospy.init_node('integration')
time.sleep(1)

publisher = rospy.Publisher("/ultralytics/detection", Image, queue_size=5)

def callback(data):
    array = ros_numpy.numpify(data)
    result = model(array)
    annotated = result[0].plot(show=False)
    msg = ros_numpy.msgify(Image, annotated, encoding='rgb8')
    publisher.publish(msg)
    

rospy.Subscriber("/camera/color/image_raw", Image, callback)

while True:
    pass