#!/home/zz/anaconda3/envs/semgrasp/bin/python3.9
import os, sys

curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(curr_dir)

import numpy as np
import open3d as o3d

from gsnet import AnyGrasp
from graspnetAPI import GraspGroup
from datetime import datetime

def anygrasp_get_poses(checkpoint_path, color, depth, text_prompt=None, model = "rs_right"):
    class Cnfg():
        def __init__(self):
            self.checkpoint_path = checkpoint_path
            self.max_gripper_width = 0.1
            self.color = color
            self.depth = depth
            self.gripper_height = 0.03
            self.top_down_grasp = True
            self.debug = True
    
    cnfg = Cnfg()
    debug = cnfg.debug
    
    anygrasp = AnyGrasp(cnfg)
    anygrasp.load_net()

    # get data
    if text_prompt is None:
        colors = np.array(cnfg.color, dtype=np.float32) / 255.0
        depths = np.array(cnfg.depth)
    else:
        colors = np.array(cnfg.color) / 255.0
        depths = np.array(cnfg.depth)
    # get camera intrinsics
    
    if model == "rs_right":
        fx, fy = 386.4509582519531, 385.8191223144531
        cx, cy = 318.2220153808594, 238.8162841796875
    elif model == "rs_left":
        fx, fy = 392.26812744140625, 392.26812744140625
        cx, cy = 325.4682312011719, 242.28213500976562
    else:
        fx, fy = 320.0, 320.0
        cx, cy = 319.5, 239.5
    
    scale = 1000.0
    # set workspace to filter output grasps
    # xmin, xmax = -0.19, 0.12
    # ymin, ymax = 0.02, 0.15
    # zmin, zmax = 0.0, 1.0
    # lims = [xmin, xmax, ymin, ymax, zmin, zmax]
    
    xmin, xmax = -0.19, 0.15
    ymin, ymax = -0.1, 0.15
    zmin, zmax = 0.0, 1.0
    lims = [xmin, xmax, ymin, ymax, zmin, zmax]

    # get point cloud
    xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depths / scale
    points_x = (xmap - cx) / fx * points_z
    points_y = (ymap - cy) / fy * points_z

    # set your workspace to crop point cloud
    mask = (points_z > 0) & (points_z < 1)
    points = np.stack([points_x, points_y, points_z], axis=-1)
    points = points[mask].astype(np.float32)
    colors = colors[mask].astype(np.float32)
    # print(points.min(axis=0), points.max(axis=0))

    gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True) 

    if len(gg) == 0:
        print('No Grasp detected after collision detection!')

    gg = gg.nms().sort_by_score()
    gg_pick = gg[0:50]
    results = []
    for i in range(len(gg_pick)):
        translation = gg_pick[i].translation
        score =  gg_pick[i].score
        rotation_matrix = gg_pick[i].rotation_matrix
        save_dict = {"trans": translation.tolist(), "score": score, "rotation_matrix": rotation_matrix.tolist()}
        results.append(save_dict)

    # visualization
    trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    cloud.transform(trans_mat)
    grippers = gg.to_open3d_geometry_list()
    for gripper in grippers:
        gripper.transform(trans_mat)
    # import pdb; pdb.set_trace()
    # o3d.io.write_point_cloud(f"/home/zz/ros_proj/erdaiji_ws/src/manipulation_ros/pointcloud/{timestr}_point_cloud_sceen.ply", cloud)
    # o3d.io.write_point_cloud(f"/home/zz/ros_proj/erdaiji_ws/src/manipulation_ros/pointcloud/{timestr}_grippers_anygrasp.ply", grippers[0])
    # import pdb
    # pdb.set_trace()
    # if debug:
    #     o3d.visualization.draw_geometries([*grippers, cloud])
    return results, cloud
    
# if __name__ == '__main__':
#     from PIL import Image_bullet
#     frame_name = "test_24-08-03-17-36-22"
#     grippers = anygrasp_get_poses(Image.open("/home/zz/Downloads/20241230_231712_depth等2个文件/20241230_231712_color.png"),
#                                   Image.open("/home/zz/Downloads/20241230_231712_depth等2个文件/20241230_231712_depth.png"),
#                                   model="rs_right")
#     print("-----------")
#     print(grippers)
