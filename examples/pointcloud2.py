import rospy
import time
from ultralytics import YOLO
import ros_numpy
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import sys
rospy.init_node('ultralytics')
time.sleep(1)
segmentation_model = YOLO("yolov8m-seg.pt")


def pointcloud2_to_array(pointcloud2: PointCloud2) -> tuple:
    """
    Convert a ROS PointCloud2 message to a numpy array
    Args:
        pointcloud2 (PointCloud2): the PointCloud2 message
    Returns:
        tuple: tuple containing: (xyz, rgb)
    """
    pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud2)
    split = ros_numpy.point_cloud2.split_rgb_field(pc_array)
    rgb = np.stack([split['b'], split['g'], split['r']], axis=2)
    xyz = ros_numpy.point_cloud2.get_xyz_points(pc_array, remove_nans=False)
    xyz = np.array(xyz).reshape((pointcloud2.height, pointcloud2.width, 3))
    nan_rows = np.isnan(xyz).all(axis=2)
    xyz[nan_rows] = [0, 0, 0]
    rgb[nan_rows] = [0, 0, 0]
    return xyz, rgb


ros_cloud = rospy.wait_for_message("/camera/depth/points", PointCloud2)
xyz, rgb = pointcloud2_to_array(ros_cloud)
result = segmentation_model(rgb)


if not len(result[0].boxes.cls):
    print("No objects detected")
    sys.exit()

classes = result[0].boxes.cls.cpu().numpy().astype(int)
for index, class_id in enumerate(classes):
    mask = result[0].masks.data.cpu().numpy()[index,:,:].astype(int)
    mask_expanded = np.stack([mask, mask, mask], axis=2)
    
    obj_rgb = rgb * mask_expanded
    obj_xyz = xyz * mask_expanded
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(obj_xyz.reshape((ros_cloud.height* ros_cloud.width, 3)))
    pcd.colors = o3d.utility.Vector3dVector(obj_rgb.reshape((ros_cloud.height* ros_cloud.width, 3)) / 255)
    o3d.visualization.draw_geometries([pcd])
