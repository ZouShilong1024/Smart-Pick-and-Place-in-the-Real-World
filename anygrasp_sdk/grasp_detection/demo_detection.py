#!/home/zz/anaconda3/envs/anygrasp/bin/python3
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
import argparse
rospy.sleep(80)
parser = argparse.ArgumentParser()
# parser.add_argument('--checkpoint_path', help='Model checkpoint path', default="/home/zz/ros_proj/manipulation_ws/src/anygrasp-ros/src/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar")
# # parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path', default="log/checkpoint_detection.tar")
# parser.add_argument('--max_gripper_width', type=float, default=0.1, help='Maximum gripper width (<=0.1m)')
# parser.add_argument('--image_dir', default='/home/zz/ros_proj/manipulation_ws/src/anygrasp-ros/src/anygrasp_sdk/grasp_detection/example_data/graspnetdata_res')
# parser.add_argument('--gripper_height', type=float, default=0.03, help='Gripper height')
# parser.add_argument('--top_down_grasp', action='store_true', help='Output top-down grasps.')
# parser.add_argument('--debug', action='store_true', help='Enable debug mode', default=True)
# cfgs = parser.parse_args()
# cfgs.max_gripper_width = max(0, min(0.1, cfgs.max_gripper_width))


class Parameter():
    def __init__(self, ):
        self.checkpoint_path = '/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/anygrasp_sdk/grasp_detection/log/checkpoint_detection_rs.tar'
        self.max_gripper_width = 0.1
        self.image_dir = '/home/zz/ros_proj/erdaiji_ws/src/camera_ros/data'
        self.gripper_height = 0.03
        self.top_down_grasp = False
        self.debug = True

param = Parameter()

def demo():
    anygrasp = AnyGrasp(param)
    anygrasp.load_net()

    # get data
    colors = np.array(Image.open(os.path.join(param.image_dir, 'color.png')), dtype=np.float32) / 255.0
    depths = np.array(Image.open(os.path.join(param.image_dir, 'depth.png')))
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
    
    # import pdb; pdb.set_trace()
    gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

    if len(gg) == 0:
        print('No Grasp detected after collision detection!')

    gg = gg.nms().sort_by_score()
    gg_pick = gg[0:20]
    print(gg_pick.scores)
    print('grasp score:', gg_pick[0].score)
    print('best pose', gg_pick[0])
    results = {}
    for i in range(len(gg_pick)):
        translation = gg_pick[i].translation
        score =  gg_pick[i].score
        rotation_matrix = gg_pick[i].rotation_matrix
        save_dict = {"trans": translation.tolist(), "score": score, "rotation_matrix": rotation_matrix.tolist()}
        results[i] = save_dict

    # with open("./result.txt", "w") as f:
    #     f.write(json.dumps(results))
    # f.clos()
    with open("./result.json", "w") as f:
        json.dump(results, f, indent=5)
    f.close()

    # visualization
    if param.debug:
        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        cloud.transform(trans_mat)
        grippers = gg.to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(trans_mat)
        print(grippers[0])
        o3d.visualization.draw_geometries([*grippers, cloud])
        o3d.visualization.draw_geometries([grippers[0], cloud])
        return grippers


if __name__ == '__main__':
    
    gs = demo()
