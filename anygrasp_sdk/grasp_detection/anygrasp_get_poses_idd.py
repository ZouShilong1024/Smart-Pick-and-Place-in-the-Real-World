#!/home/zz/anaconda3/envs/anygrasp/bin/python3.9
import os
import argparse
import torch
import json
import numpy as np
import open3d as o3d
from PIL import Image
import rospy

from gsnet import AnyGrasp
from graspnetAPI import GraspGroup


pkg_dir = "/home/zz/ros_proj/manipulation_ws/src/anygrasp-ros"

def create_directory(path):
    try:
        # Create the directory
        os.makedirs(path)
        print(f"Directory created at {path}")
    except FileExistsError:
        pass
        # print(f"Directory already exists at {path}")
    except Exception as e:
        print(f"An error occurred while creating directory at {path}: {e}")



def get_poses(frame_name):
    class Cnfg():
        def __init__(self, frame_name):
            self.checkpoint_path = "{}/src/log/checkpoint_detection_rs.tar".format(pkg_dir)
            self.max_gripper_width = 0.1
            self.frame_path = "{}/src/frames_saved/{}/".format(pkg_dir, frame_name)
            self.gripper_height = 0.03
            self.top_down_grasp = False
            self.debug = True
    
    cnfg = Cnfg(frame_name)
    frame_path = cnfg.frame_path
    debug = cnfg.debug
    
    
    save_path = frame_path.replace("frames", "results")
    create_directory(save_path)
    
    
    anygrasp = AnyGrasp(cnfg)
    anygrasp.load_net()

    # get data
    colors = np.array(Image.open(os.path.join(frame_path, 'color.png')), dtype=np.float32) / 255.0
    depths = np.array(Image.open(os.path.join(frame_path, 'depth.png')))
    # get camera intrinsics
    # fx, fy = 641.712, 622.06
    # cx, cy = 622.06, 354.86
    # scale = 1000.0
    fx, fy = 641.543212890625, 640.9203491210938
    cx, cy = 622.057861328125, 354.8597412109375
    scale = 1000.0
    # set workspace to filter output grasps
    xmin, xmax = -0.19, 0.12
    ymin, ymax = 0.02, 0.15
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
    print(points.min(axis=0), points.max(axis=0))

    gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

    if len(gg) == 0:
        print('No Grasp detected after collision detection!')

    gg = gg.nms().sort_by_score()
    gg_pick = gg[0:20]
    print(gg_pick.scores)
    print('grasp score:', gg_pick[0].score)
    print('best pose', gg_pick[0])
    results = []
    for i in range(len(gg_pick)):
        translation = gg_pick[i].translation
        score =  gg_pick[i].score
        rotation_matrix = gg_pick[i].rotation_matrix
        save_dict = {"trans": translation.tolist(), "score": score, "rotation_matrix": rotation_matrix.tolist()}
        results.append(save_dict)

    # with open("./result.txt", "w") as f:
    #     f.write(json.dumps(results))
    # f.clos()
    with open(save_path + "result.json", "w") as f:
        json.dump(results, f, indent=5)
    f.close()

    # visualization
    trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    cloud.transform(trans_mat)
    grippers = gg.to_open3d_geometry_list()
    for gripper in grippers:
        gripper.transform(trans_mat)
    print(grippers[0])
    if debug:
        o3d.visualization.draw_geometries([*grippers, cloud])
        o3d.visualization.draw_geometries([grippers[0], cloud])
    return grippers
    

if __name__ == '__main__':
    frame_name = "test_24-08-03-17-36-22"
    grippers = get_poses(frame_name)
    print("-----------")
    print(grippers)
