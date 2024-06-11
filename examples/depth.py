import rospy
import time
from ultralytics import YOLO
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
rospy.init_node('integration')
time.sleep(1)

segmentation_model = YOLO("yolov8m-seg.pt")

classes_pub = rospy.Publisher("/ultralytics/detection/distance", String, queue_size=5)

def callback(data):
    image = rospy.wait_for_message("/camera/color/image_raw", Image)
    image = ros_numpy.numpify(image)
    depth = ros_numpy.numpify(data)
    result = segmentation_model(image)
    
    all_objects = []
    for index, cls in enumerate(result[0].boxes.cls):
        class_index = int(cls.cpu().numpy())
        name = result[0].names[class_index]
        mask = result[0].masks.data.cpu().numpy()[index,:,:].astype(int)
        obj = depth[mask == 1]
        obj = obj[~np.isnan(obj)]
        avg_distance = np.mean(obj) if len(obj) else np.inf
        all_objects.append((name, avg_distance))
        
    msg = String()
    msg.data = str(all_objects)
    classes_pub.publish(msg)

rospy.Subscriber("/camera/depth/image_raw", Image, callback)

while True:
    rospy.spin()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
# for mask in result[0].masks.data.cpu().numpy()[:,:,:].astype(int):
#     cv2.imshow("depth", depth * mask)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()


# mask = result[0].masks.data.cpu().numpy()[0,:,:].astype(int)
# mask_expanded = mask[:, :, np.newaxis]
# # Apply the mask to the image
# masked_image = image * mask_expanded
# masked_image = masked_image.astype(np.uint8)
# # res = cv2.bitwise_and(image,image,mask = mask)

# sidebyside = np.hstack([result[0].plot(show=False), masked_image])

# cv2.imshow("depth", sidebyside)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

