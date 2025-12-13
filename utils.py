import time
import numpy as np
import open3d as o3d
from graspnetAPI import GraspGroup

def graspcam2pixel(grasping_pose, type="right"):
    grasp_points = []
    for grasp in grasping_pose:
        translation = np.array(grasp['trans'])
        grasp_point = translation.reshape((-1, 3))
        grasp_points.append(grasp_point)
    
    grasp_points = np.concatenate(grasp_points, axis=0)

    X_c, Y_c, Z_c = grasp_points[:, 0], grasp_points[:, 1], grasp_points[:, 2]
    if type == "right":
        fx, fy = 385.9778137207031, 385.34674072265625
        cx, cy = 318.2220153808594, 238.8162841796875
    else:
        fx, fy = 392.26812744140625, 392.26812744140625
        cx, cy = 325.4682312011719, 242.28213500976562
    u = (fx * X_c / Z_c) + cx
    v = (fy * Y_c / Z_c) + cy
    points_screen = np.vstack((u, v)).T
    
    return points_screen[:, :2], grasping_pose

def pixel_to_camera_point(pixel_points, depth_image, type="right"):
    if type == "right":
        fx, fy = 385.9778137207031, 385.34674072265625
        cx, cy = 318.2220153808594, 238.8162841796875
    else:
        fx, fy = 392.26812744140625, 392.26812744140625
        cx, cy = 325.4682312011719, 242.28213500976562
    u_points = pixel_points[:, 0].astype(np.int16)
    v_points = pixel_points[:, 1].astype(np.int16)
    depth_values = depth_image[v_points, u_points] 
    grasp_points_cam = []
    for u, v, depth_value in zip(pixel_points[:, 0], pixel_points[:, 1], depth_values):
        Z_c = depth_value
        X_c = (u - cx) * Z_c / fx
        Y_c = (v - cy) * Z_c / fy
        grasp_points_cam.append([X_c, Y_c, Z_c])
    grasp_points_3d_m = np.stack(grasp_points_cam, axis=0) * 1e-3

    return grasp_points_3d_m

def pixel_to_camera_point2(pixel_points, depth_value_m, type="right"):
    if type == "right":
        fx, fy = 385.9778137207031, 385.34674072265625
        cx, cy = 318.2220153808594, 238.8162841796875
    else:
        fx, fy = 392.26812744140625, 392.26812744140625
        cx, cy = 325.4682312011719, 242.28213500976562

    u_points = pixel_points[:, 0]
    v_points = pixel_points[:, 1]
    if isinstance(depth_value_m, (int, float)):
        Z_c = np.full_like(u_points, depth_value_m, dtype=np.float32)
    else:
        Z_c = depth_value_m
    X_c = (u_points - cx) * Z_c / fx
    Y_c = (v_points - cy) * Z_c / fy
    grasp_points_3d_m = np.stack([X_c, Y_c, Z_c], axis=1)

    return grasp_points_3d_m

def self_rotation_np(pose):
    transformation_matrix = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    pose = transformation_matrix @ pose
    return pose

def rpy_to_vector(r, p, y, axis = [1,0,0]):
    """
    Converts roll, pitch, and yaw to a unit vector.
    The resulting vector corresponds to the direction of the pose in 3D space.
    """
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    rotation_matrix = Rz @ Ry @ Rx
    direction_vector = rotation_matrix @ np.array(axis)
    
    return direction_vector

def transform_world_to_camera(pose_matrix, T_base_to_cam):        
    T_base_to_cam_inv = np.linalg.inv(T_base_to_cam)
    transformed_pose = T_base_to_cam_inv @ pose_matrix
    return transformed_pose

def self_rotation_inv(pose):

    transformation_matrix = np.array([[0, 0, 1, 0],
                                     [1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 0, 1]])
    transformation_matrix = np.array([[ 0,  0,  1,  0],
       [ 0,  1,  0,  0],
       [-1,  0,  0,  0],
       [ 0,  0,  0,  1]])
    inverse_matrix = np.linalg.inv(transformation_matrix)
    return inverse_matrix[:3, :3]


def visualization(cloud, grasp_pose):
    # cloud = o3d.io.read_point_cloud("/home/zz/ros_proj/manipulation_ws/src/visual/data/point_cloud.ply")
    # grasp_pose = np.loadtxt("/home/zz/ros_proj/manipulation_ws/src/visual/data/grasping_pose.txt")
    gg = np.array([[0.17656013369560242,
                0.0575287826359272,
                0.029999999329447746,
                0.029999999329447746,
                grasp_pose[0][0], grasp_pose[0][1], grasp_pose[0][2],
                grasp_pose[1][0], grasp_pose[1][1], grasp_pose[1][2],
                grasp_pose[2][0], grasp_pose[2][1], grasp_pose[2][2],
                grasp_pose[0][3], grasp_pose[1][3], grasp_pose[2][3],
                -1]], dtype = np.float64)
    trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
    grippers = GraspGroup(gg).to_open3d_geometry_list()
    cloud = cloud.transform(trans_mat)
    gripper_pose = grippers[0].transform(trans_mat)
    gripper_pose.paint_uniform_color([0, 0, 1])  
    o3d.visualization.draw_geometries([grippers[0], cloud])
    # import pdb; pdb.set_trace()

    # cloud = cloud.transform(trans_mat)
    # gripper_pose = grippers[0].transform(trans_mat)
    # gripper_pose.paint_uniform_color([0, 0, 1])
    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(cloud)
    # vis.add_geometry(gripper_pose)
    # vis.poll_events()
    # vis.update_renderer()
    # time.sleep(10)

    # o3d.io.write_triangle_mesh(f"/home/zz/ros_proj/erdaiji_ws/src/manipulation_ros/pointcloud/{timestamp}_best_gripper.ply", grippers[0])
    # o3d.io.write_point_cloud(f"/home/zz/ros_proj/erdaiji_ws/src/manipulation_ros/pointcloud/{timestamp}_point_cloud.ply", cloud)