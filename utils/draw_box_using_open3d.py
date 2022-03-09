import open3d as o3d
from .box_util import flip_axis_to_camera_np
import numpy as np
import matplotlib.pyplot as plt



# color_map = {'black': np.array([0,0,0]),
#              'red': np.array([255,0,0]),
#              'blue': np.array([0,0,255]),
#              'green': np.array([0,255,0]),
#              'purple': np.array([153, 0, 153]),
#              'mata': np.array([0, 153, 153]),
#              'brown': np.array([255,255,0]),
#              'sky': np.array([204,0,153]),
#              'gold': np.array([153,51,0])}
#              'white'

# "bed": 0,
# "table": 1,
# "sofa": 2,
# "chair": 3,
# "toilet": 4,
# "desk": 5,
# "dresser": 6,
# "night_stand": 7,
# "bookshelf": 8,
# "bathtub": 9,
color_map = {0: np.array([0, 0, 0]),
                 1: np.array([255, 0, 0]),
                 2: np.array([0, 0, 255]),
                 3: np.array([0, 255, 0]),
                 4: np.array([153, 0, 153]),
                 5: np.array([0, 153, 153]),
                 6: np.array([102, 51, 0]),
                 7: np.array([51, 255, 255]),
                 8: np.array([255, 204, 0]),
                 9: np.array([0,0,0])}


def drawBox(_pointcloud, _predicted_boxes, _gt_boxes):


    pointcloud = _pointcloud.cpu().detach().numpy()
    # predicted_boxes = _predicted_boxes.cpu().detach().numpy()
    gt_boxes = _gt_boxes.cpu().detach().numpy()

    pointcloud = flip_axis_to_camera_np(pointcloud[0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)

    d = o3d.geometry.OrientedBoundingBox()
    objects = []

    objects.append(pcd)

    for key, box in _predicted_boxes.items():
        for index in range(len(_predicted_boxes[key][0])):
            is_all_zero = np.all((_predicted_boxes[key][0][index] == 0))
            if is_all_zero:
                 continue
            b = d.create_from_points(o3d.utility.Vector3dVector(np.array(_predicted_boxes[key][0][index])))
            b.color = color_map[key]
            objects.append(b)


    o3d.visualization.draw_geometries(objects)

