#!/usr/bin/env python3


import os, sys
import rospy
import numpy as np
import math
from collections import namedtuple
import json
import time
import pybullet as p
import pybullet_data
import threading
import copy


# script_dir = os.path.dirname(os.path.abspath(__file__))
# parent_dir = os.path.dirname(script_dir)
# sys.path.append(script_dir)
# sys.path.append(parent_dir)
from utils import load_json_file,  draw_axis, calcul_transformation_matrix, calcul_pos_orn 
from p_utils import get_self_link_pairs
# from camera import Camera
from itertools import combinations


ROUND_NUMBER = 3



def world_to_robot(robot_id, world_pos, world_orn):
    robot_base_pos, robot_base_orn = p.getLinkState(robot_id, 0)[:2] 
    # print(robot_id)
    # print("robot_base_pos, robot_base_orn")
    # print(robot_base_pos, robot_base_orn)
    # print("=---------------------------")
    # pos, orn = p.getLinkState(robot_id, 0)[:2] 
    # print("pos, orn")
    # print(pos, orn)
    # print("===========================")
    T_w_r = calcul_transformation_matrix(robot_base_pos, robot_base_orn)
    T_w_p = calcul_transformation_matrix(world_pos, world_orn)
    T_r_p = np.linalg.inv(T_w_r) @ T_w_p
    robot_pos, robot_orn = calcul_pos_orn(T_r_p)
    return robot_pos, robot_orn
    

def robot_to_world(robot_id, robot_pos, robot_orn):
    robot_base_pos, robot_base_orn = p.getLinkState(robot_id, 0)[:2] 
    # print(robot_base_pos, robot_base_orn)
    T_w_r = calcul_transformation_matrix(robot_base_pos, robot_base_orn)
    T_r_p = calcul_transformation_matrix(robot_pos, robot_orn)
    T_w_p = T_w_r @ T_r_p
    world_pos, world_orn = calcul_pos_orn(T_w_p)
    return world_pos, world_orn


def visualize_link(robot_id, link_id,duration=100):
    pos, orn = p.getLinkState(robot_id, link_id)[:2]
    T_mat = calcul_transformation_matrix(pos, orn)
    draw_axis(T_mat,duration=duration)


class ErdaijiRobot:
    
    def __init__(self, pos, ori, robot_path = None, config_path = None, fixed_robot = False, blocking_mode = True, vis = True):
        self.base_pos = pos
        self.base_ori = p.getQuaternionFromEuler(ori)
        self.robot_path =robot_path
        self.config_path = config_path
        self.robot_config = load_json_file(self.config_path)
        # self.camera_config = load_json_file(self.config_path.replace('robot_config', 'camera_config'))
        self.fixed_robot = fixed_robot
        self.blocking_mode = blocking_mode
        self.self_collision_threshold = 0.00
        self.collision_threshold = 0.001
        self.vis = vis
        
        del self.robot_config['left_hand']
        self.struct_list = list(self.robot_config.keys())
        rospy.loginfo(f"ROBOT STRUCT LIST: {self.struct_list}")
        
        self.default_poses = {}
        for struct_name in self.struct_list:
            self.default_poses[struct_name] = self.robot_config[struct_name]["default_pose"]
    
    
    def load_robot(self):
        # >>> load robot model
        self.id = p.loadURDF(self.robot_path, self.base_pos, self.base_ori,
                             useFixedBase=self.fixed_robot, 
                             flags=
                                p.URDF_USE_SELF_COLLISION 
                                # p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES 
                                # | p.URDF_USE_SELF_COLLISION 
                                # | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS 
                                # | p.URDF_MERGE_FIXED_LINKS
                             )
        # >>> create agv constraint
        self.agv = AGV(self.id, self.base_pos, self.base_ori, vis = self.vis)
        self.agv.reset_agv()
        rospy.loginfo(f"ROBOT URDF LOADED")
        
        self.__parse_joint_info__()
        self.__post_load__()
        
        
        # self.set_up_camera()
                                     
        
        
        return True
    def set_up_camera(self):
        # <<< load camera params and set up cameras
        self.cameras = []
        print(self.camera_config)
        for i in range(len(self.camera_config)):
            mount_struct = self.camera_config[i]["struct"]
            mount_link = self.camera_config[i]["mount_link"]
            mount_link_id = self.linkName_to_id[mount_link]
            T_mount = self.camera_config[i]["Ts"]
            camera = Camera("dk", robot_mount = self.id, link_mount = [mount_link, mount_link_id], T_mount = T_mount, use_array = True)
            self.cameras.append(camera)
        self.camera_visualize = self.cameras[-1]
        self.update_camera()
    
    def select_camera(self, idx):
        assert idx >= 0 and idx < len(self.cameras)
        self.camera_visualize = self.cameras[idx]
        return self.camera_visualize
    
    def __parse_joint_info__(self):
        self.numJoints = p.getNumJoints(self.id)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.jointname_to_id = {}
        self.linkName_to_id = {}
        self.controllable_joints = []
        self.controllable_joints_name = []
        for i in range(self.numJoints):
            info = p.getJointInfo(self.id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            linkName = info[12].decode("utf-8")
            controllable = (jointType != p.JOINT_FIXED)
            self.jointname_to_id[jointName] = i
            self.linkName_to_id[linkName] = i
            if controllable:
                self.controllable_joints.append(jointID)
                self.controllable_joints_name.append(jointName)
                p.setJointMotorControl2(self.id, jointID, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)
        

        # self.controllable_lower_limits = [info.lowerLimit for info in self.joints if info.controllable]
        # self.controllable_upper_limits = [info.upperLimit for info in self.joints if info.controllable]
        # self.controllable_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable]
        self.controllable_lower_limits = {}
        self.controllable_upper_limits = {}
        self.controllable_joint_ranges = {}
        for info in self.joints:
            if info.controllable:
                self.controllable_lower_limits[info.name] = info.lowerLimit
                self.controllable_upper_limits[info.name] = info.upperLimit
                self.controllable_joint_ranges[info.name] = info.upperLimit - info.lowerLimit
        
        self.robot_joints_info ={
            "names":self.controllable_joints_name,
            "ids":self.controllable_joints,
            "lower_limits":self.controllable_lower_limits,
            "upper_limits":self.controllable_upper_limits,
            "joint_ranges":self.controllable_joint_ranges
        }
        
        for a, b in zip(self.controllable_joints, self.controllable_joints_name):
            rospy.loginfo(f"CONTROLLABLE JOINTS IDS: {a}          CONTROLLABLE JOINTS NAMES: {b}")
            print((f"CONTROLLABLE JOINTS IDS: {a}          CONTROLLABLE JOINTS NAMES: {b}"))
        for v, k in self.linkName_to_id.items():
            rospy.loginfo(f"Link IDS: {k}          Link NAMES: {v}")
            print(f"Link IDS: {k}          Link NAMES: {v}")

        
    
    
    def __post_load__(self):
        
        self.robot_structs = {}
        for struct_name in self.struct_list:
            struct_config = {
                "robot_id": self.id,
                "struct_name": struct_name,
                "joint_names": self.robot_config[struct_name]["joint_names"], 
                "joint_ids": [self.jointname_to_id[j] for j in self.robot_config[struct_name]["joint_names"]],
                "link_names": self.robot_config[struct_name]["link_names"], 
                "link_ids": [self.linkName_to_id[l] for l in self.robot_config[struct_name]["link_names"]],
                "joint_infos": [self.joints[self.jointname_to_id[j]] for j in self.robot_config[struct_name]["joint_names"]],
                
                "controllable_joint_names": self.robot_config[struct_name]["controllable_joint_names"], 
                "controllable_joint_ids": [self.jointname_to_id[j] for j in self.robot_config[struct_name]["controllable_joint_names"]],
                
                "end_link": self.robot_config[struct_name]["end_link"],
                "end_link_id": self.linkName_to_id[self.robot_config[struct_name]["end_link"]],
                "default_pose": self.default_poses[struct_name],
                "mimic":self.robot_config[struct_name]["mimic"],
                "lower_limits": [self.controllable_lower_limits[joint] for joint in self.robot_config[struct_name]["joint_names"]], 
                "upper_limits": [self.controllable_upper_limits[joint] for joint in self.robot_config[struct_name]["joint_names"]], 
                "joint_ranges": [self.controllable_joint_ranges[joint] for joint in self.robot_config[struct_name]["joint_names"]],
                "robot_joints_info":self.robot_joints_info
            }
            self.robot_structs[struct_name] = TYPE_MAPPER[self.robot_config[struct_name]["struct_type"]](struct_config, blocking_mode = self.blocking_mode, vis = self.vis)
            rospy.loginfo(f"ROBOT STRUCTURE {struct_name.upper()} DEFINED:")

        
        # construct collision pairs
        ignore_pair_set = set()
        for struct_name in self.struct_list:
            if self.robot_config[struct_name]["struct_type"] == "hand":
                ig_set = set(combinations([self.linkName_to_id[l] for l in self.robot_config[struct_name]["link_names"]], 2))
                ignore_pair_set |= ig_set
        # print(ignore_pair_set)
        
        self.link_pairs = get_self_link_pairs(
            self.id,
            self.controllable_joints, 
            disabled_collisions= ignore_pair_set
            )
        # for p in self.link_pairs:
        #     print(p)
        
        
        rospy.loginfo(f"ALL ROBOT STRUCTURES DEFINED")
        rospy.loginfo(f"================================================")
        
        # <<< add joint mimic constraints
        # need_mimics_list = [self.left_gripper.mimic]#[self.left_gripper.mimic, self.right_hand.mimic]
        # for mimic in need_mimics_list:
        #     for mimic_parent_name, mimic_children_names in mimic.items():
        #         self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)
        # rospy.loginfo(f"ROBOT MIMIC JOINTS CREATED")
        # <<< add FT sensor 
        # need_FT_list = [self.left_arm.end_link_id, self.right_arm.end_link_id]
        # for id in need_FT_list:
        #     self.enable_joint_FT(id)
        # rospy.loginfo(f"ROBOT FORCE TORQUE SENSOR CREATED")
        
        return True
        
    # def __setup_mimic_joints__(self, mimic_parent_name, mimic_children_names):
    #     self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
    #     self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if joint.name in mimic_children_names}

    #     for joint_id, multiplier in self.mimic_child_multiplier.items():
    #         c = p.createConstraint(self.id, self.mimic_parent_id,
    #                                self.id, joint_id,
    #                                jointType=p.JOINT_GEAR,
    #                                jointAxis=[0, 1, 0],
    #                                parentFramePosition=[0.0, 0.0, 0.0],
    #                                childFramePosition=[0.0, 0.0, 0.0])
    #         p.changeConstraint(c, gearRatio=-multiplier, maxForce=400, erp=1)  # Note: the mysterious `erp` is of EXTREME importance

    # def enable_joint_FT(self, joint_index: int = 3):
        # p.enableJointForceTorqueSensor(bodyUniqueId=self.id, jointIndex=joint_index, enableSensor=True)

    
    
    def reset_robot(self):
        self.agv.reset_agv()
        for struct_name, struct in self.robot_structs.items():
            struct.reset() 
        # self.update_camera()

        return True
    
    def get_state(self):
        state = {}
        state["agv"] = self.agv.get_agv_state()
        state["js"] = {}
        for struct_name in self.struct_list:
            state["js"][struct_name] = self.robot_structs[struct_name].get_joint_pose()
        
        state["ee_world"] = {}
        for struct_name in self.struct_list:
            state["ee_world"][struct_name] = self.robot_structs[struct_name].get_end_link_pose_world()
        
        state["ee_self"] = {}
        for struct_name in self.struct_list:
            state["ee_self"][struct_name] = self.robot_structs[struct_name].get_end_link_pose_self()
            
        return state
    def apply_actions(self):

        self.agv.execute_cmd()
        for struct_name in self.struct_list:
            self.robot_structs[struct_name].execute_cmd()
        
    
    def receive_cmds(self, cmds):
        if cmds is None:
            return False
        for struct_name, cmd in cmds.items():
            if struct_name in self.robot_structs:
                self.robot_structs[struct_name].receive_cmd(cmd)
            elif struct_name == "agv":
                self.agv.receive_cmd(cmd)
        return True
    
    def update_camera(self):
        self.camera_stream = self.camera_visualize.update_img()
        return self.camera_stream

    def get_camera_image(self, idx):
        assert idx >= 0 and idx < len(self.cameras)
        self.camera_visualize = self.cameras[idx]
        return self.update_camera()

    def get_camera_stream(self):
        return self.camera_stream
    
    def check_collision(self):
        # <<< self collision
        closest_points_to_self = [
            p.getClosestPoints(
                bodyA=self.id, bodyB=self.id,
                distance=self.self_collision_threshold,
                linkIndexA=link1, linkIndexB=link2)
            for link1, link2 in self.link_pairs]
        for closest_points_to_self_link in closest_points_to_self:
            for point in closest_points_to_self_link:
                if len(point) > 0:
                    linka = point[3]
                    linkb = point[4]
                    for key, val in self.linkName_to_id.items():
                        if val == linka:
                            namea = key
                        if val == linkb:
                            nameb = key
                    rospy.logwarn(f"COLLISION FOUND IN [SELF] COLLISION CHECK: {namea} AND {nameb}")
            for point in closest_points_to_self_link:
                if len(point) > 0:
                    self.prev_collided_with = point
                    return True, self.id
        
        # collision with others
        others_id = [p.getBodyUniqueId(i)
                     for i in range(p.getNumBodies())
                     if p.getBodyUniqueId(i) != self.id]
        
        closest_points_to_others = [
            sorted(list(p.getClosestPoints(
                bodyA=self.id, bodyB=other_id,
                distance=self.collision_threshold)),
                key=lambda contact_points: contact_points[8])
            if other_id != 0 else []
            for other_id in others_id]
        for i, closest_points_to_other in enumerate(
                closest_points_to_others):
            if len(closest_points_to_other) > 0:
                for point in closest_points_to_other:
                    if point[8] < self.collision_threshold:
                        self.prev_collided_with = point
                        rospy.logwarn(f"COLLISION FOUND IN [external] COLLISION CHECK: ROBOT AND {point[2]}")
                        return True, point[2]
        return False, None
        



class AGV:
    def __init__(self, robot_id, base_pos, base_ori, blocking_mode = True, vis = True):
        self.id = robot_id
        self.agv_cid = None
        
        self.base_pos = base_pos
        self.base_ori = base_ori
        
        self.update_agv_pose()
        self.curr_pos = self.get_agv_state()
        self.prev_pos = copy.deepcopy(self.curr_pos)
        
        self.cmd_queue = []
        self.count_execute_cmd = 0
        
        self.step_length = 0.08
        
        self.blocking_mode = blocking_mode
        self.block_threshold =0.001
        self.achievement_threshold = 0.05
        self.fail_step_threshold = 10000
        self.vis = vis
    
    def update_agv_pose(self):
        self.agv_pos, self.agv_ori = p.getLinkState(self.id, 0)[:2]
        return self.agv_pos, self.agv_ori
    def get_agv_state(self):
        pos, ori = self.update_agv_pose()
        x = pos[0]
        y = pos[1]
        theta = p.getEulerFromQuaternion(ori)[-1]
        return np.array([x, y, theta])
    def reset_agv(self):
        if self.agv_cid is not None:
            p.removeConstraint(self.agv_cid)
        self.agv_cid = p.createConstraint(self.id, 0, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], self.base_pos, parentFrameOrientation = self.base_ori)
        
        self.update_agv_pose()
        
        self.curr_pose = self.get_agv_state()
        self.prev_pose = copy.deepcopy(self.curr_pose)
        
        return self.agv_cid
    def move_agv(self, agv_goal):
        x = agv_goal[0]
        y = agv_goal[1]
        theta = agv_goal[2]
        pivot = [x, y, 0]
        orn = p.getQuaternionFromEuler([0,0,theta])
        p.changeConstraint(self.agv_cid, pivot, jointChildFrameOrientation=orn, maxForce=10000)


    def receive_cmd(self, cmd):
        if "type" not in cmd or "act" not in cmd:
            rospy.logwarn("NOT A CORRECT CMD, REJECT")
            return False
        else:
            self.cmd_queue.append(cmd)
            # rospy.loginfo(f"CMD RECEIVED. {self.struct_name}:  {cmd}")
            return True
    
    def check_current_cmd(self):
        try:
            return copy.deepcopy(self.cmd_queue[0])
        except:
            return None
    def execute_cmd(self):
        if len(self.cmd_queue) == 0:
            return False
        self.count_execute_cmd += 1
        self.prev_pose = copy.deepcopy(self.curr_pose)
        self.curr_pose = self.get_agv_state()
        
        cmd_to_execute =self.cmd_queue[0]
        
        check = self.get_action_and_execute(cmd_to_execute)
        
        self.check_blocking_mode()
        return check
    def check_cmd_achievement(self, cmd):
        if "type" not in cmd or "act" not in cmd:
            rospy.logwarn("NOT A CORRECT CMD, REJECT")
            return False, {}
        curr_pose = self.get_agv_state()
        if cmd["type"] == "marker":
            target = np.array([0,0,0])
        else:
            target = np.array(cmd["act"])
        
        delta_value = np.linalg.norm(curr_pose - target)
        check = delta_value < self.achievement_threshold
        return check, {"delta_xy":np.linalg.norm(curr_pose[:2] - target[:2]), "delta_theta":np.linalg.norm(curr_pose[2] - target[2])}
    
    def get_action_and_execute(self, cmd_to_execute):
        if cmd_to_execute["type"] == "marker":
            pose = np.array([0,0,0])
        elif cmd_to_execute["type"] == "pos":
            pose = cmd_to_execute["act"]
        else:
            cmd_type = cmd_to_execute["type"]
            rospy.logwarn(f"STRUCT agv DOSE NOT SUPPORT CMD TYPE :{cmd_type}")
            return False
        act = self.calcul_act_by_pose(pose)
        self.move_agv(act)
        return True
    
    def calcul_act_by_pose(self, target_pose):
        
        
        delta_pose = target_pose - self.curr_pose
        xy_value = np.linalg.norm(delta_pose[:2])
        
        facing_theta = np.arctan2(target_pose[1] - self.curr_pose[1], target_pose[0] - self.curr_pose[0])
        theta_value = np.linalg.norm(facing_theta - self.curr_pose[2])
        theta_value2 = np.linalg.norm(delta_pose[2])
        move_pose = None
        
        if xy_value > 0.05 and theta_value > 0.1:
            # <<< agv rotatte to the direction of target position
            delta_move_theta = min(abs(facing_theta - self.curr_pose[2]), self.step_length) * (facing_theta - self.curr_pose[2]) / abs(facing_theta - self.curr_pose[2])
            move_pose = self.curr_pose + np.array([0,0,delta_move_theta])
        
        elif xy_value > 0.05 and theta_value <= 0.1:
            # <<< agv move directly to the position
            delta_move_x = min(self.step_length, abs(target_pose[0] - self.curr_pose[0])) * (target_pose[0] - self.curr_pose[0]) / abs(target_pose[0] - self.curr_pose[0])
            delta_move_y = min(self.step_length, abs(target_pose[1] - self.curr_pose[1])) * (target_pose[1] - self.curr_pose[1]) / abs(target_pose[1] - self.curr_pose[1])
            delta_move_theta = min(abs(facing_theta - self.curr_pose[2]), self.step_length) * (facing_theta - self.curr_pose[2]) / abs(facing_theta - self.curr_pose[2])
            move_pose = self.curr_pose + np.array([delta_move_x, delta_move_y, delta_move_theta])
        else:
            # <<< agv finetune the position and rotation
            delta_move_x = min(self.step_length, abs(target_pose[0] - self.curr_pose[0])) * (target_pose[0] - self.curr_pose[0]) / abs(target_pose[0] - self.curr_pose[0])
            delta_move_y = min(self.step_length, abs(target_pose[1] - self.curr_pose[1])) * (target_pose[1] - self.curr_pose[1]) / abs(target_pose[1] - self.curr_pose[1])
            delta_move_theta = min(self.step_length, abs(target_pose[2] - self.curr_pose[2])) * (target_pose[2] - self.curr_pose[2]) / abs(target_pose[2] - self.curr_pose[2])
            move_pose = self.curr_pose + np.array([delta_move_x, delta_move_y, delta_move_theta])
        return move_pose
        
    
    
    def check_blocking_mode(self):
        if not self.blocking_mode:
            self.cmd_queue.pop(0)
        else:
            # <<< not in blocking_mode: cmd will not be poped from cmd_queue until the execution is finished(struct finished moving )
            delta_joint_value = np.linalg.norm(self.curr_pose - self.prev_pose)
            # rospy.loginfo(f" agv delta_state_value:   {delta_joint_value}")
            if self.count_execute_cmd >=3 and delta_joint_value < self.block_threshold:
                # action movement stopped, release blocking_mode
                self.cmd_queue.pop(0)
                self.count_execute_cmd = 0
                if self.vis:
                    rospy.loginfo(f"AGV  MOVE FINISHED , ")
                    rospy.loginfo(f"current_JS: {self.curr_pose}  ")
                    rospy.loginfo(f"speed(joint_value):   {delta_joint_value}")
            elif self.count_execute_cmd > self.fail_step_threshold:
                # takes to long to execute the cmd, treate is as a failure execution, release blocking_mode
                act = self.cmd_queue[0]["act"]
                rospy.logwarn(f"IT TAKES TO LONG TIME TO EXECUTING ACTION. agv:  {act}, STOP EXECUTION")
                self.cmd_queue.pop(0)
                self.count_execute_cmd = 0
        return True



class BaseStruct:
    def __init__(self, config, blocking_mode = True, vis = True):
        
        self.vis = vis
        
        self.robot_id = config["robot_id"]
        self.struct_name = config["struct_name"]
        self.joint_names = config["joint_names"]
        self.joint_ids = config["joint_ids"]
        self.link_names = config["link_names"]
        self.link_ids = config["link_ids"]
        self.joint_infos = config["joint_infos"]
        
        
        self.controllable_joint_names = config["controllable_joint_names"]
        self.controllable_joint_ids = config["controllable_joint_ids"]
        self.num_controllable_joint = len(self.controllable_joint_names)
        
        self.end_link = config["end_link"]
        self.end_link_id = config["end_link_id"]
        
        self.mimic = config["mimic"]
        self.default_pose = config["default_pose"]
        self.lower_limits = config["lower_limits"]
        self.upper_limits = config["upper_limits"]
        self.joint_ranges = config["joint_ranges"]
        self.robot_joints_info = config["robot_joints_info"]
        
        self.joint_index = [self.robot_joints_info["names"].index(j_name) for j_name in self.controllable_joint_names]

        assert len(self.controllable_joint_ids) == self.num_controllable_joint
        assert len(self.controllable_joint_ids) == self.num_controllable_joint
        
        self.maxvel = 100
        
        self.blocking_mode = blocking_mode
        self.block_threshold =0.005
        self.fail_step_threshold = 200
        self.achievement_threshold = 0.02
        self.cmd_queue = []
        self.cmd_queue.append({"type":"js", "act":copy.copy(self.default_pose["reset"])})
        self.count_execute_cmd = None
        
        if self.vis:
            visualize_link(self.robot_id, self.end_link_id)
        
        
    def reset(self):
        self.count_execute_cmd = 0
        self.reset_by_default_pose("reset")
        self.curr_js_pose = self.get_joint_pose()
        self.prev_js_pose = copy.deepcopy(self.curr_js_pose)
        return True
    
    def reset_by_joint_states(self, js):
        assert len(js) == len(self.controllable_joint_names)
        
        for i, joint_id in enumerate(self.controllable_joint_ids):
            p.resetJointState(self.robot_id, self.controllable_joint_ids[i], js[i])

            if self.joint_names[self.joint_ids.index(joint_id)] in self.mimic:
                for mimic_name, mimic_ratio in self.mimic[self.joint_names[self.joint_ids.index(joint_id)]].items():
                    mimic_id = self.joint_ids[self.joint_names.index(mimic_name)]
                    p.resetJointState(self.robot_id, mimic_id, js[i] * mimic_ratio)
        return True
    
    def reset_by_default_pose(self, pose_name):
        if pose_name not in self.default_pose:
            return False
        reset_pose = self.default_pose[pose_name]
        for i in range(self.num_controllable_joint):
            p.resetJointState(self.robot_id, self.controllable_joint_ids[i], reset_pose[i])
        return True
    
    def get_joint_pose(self):
        # return current joint value
        
        joint_state =np.array([0.0] * self.num_controllable_joint)
        for j, joint_id in enumerate(self.controllable_joint_ids):
            value, vel, _, _ = p.getJointState(self.robot_id, joint_id)
            joint_state[j] = value
        return joint_state
    
    def get_end_link_pose_world(self):
        # <<< return current end_link_pose in <<< WORLD COORDINATE >>>
        # <<< nend_link is predefined in the struct config
        return p.getLinkState(self.robot_id, self.end_link_id)[:2]    
    def get_end_link_pose_self(self):
        # <<< return current end_link_pose in <<< ROBOT COORDINATE >>>
        # <<< end_link is predefined in the struct config
        w_pos, w_orn = p.getLinkState(self.robot_id, self.end_link_id)[:2]
        return  world_to_robot(self.robot_id, w_pos, w_orn)
    
    
    def clip_by_joint_limit(self, value, joint_name):
        lower_limit = self.lower_limits[self.joint_names.index(joint_name)] #+ 0.01
        upper_limit = self.upper_limits[self.joint_names.index(joint_name)] #- 0.01
        return np.clip(value, lower_limit, upper_limit)
    
    
    def move_joint(self, joint_target):
        # <<< input target joint pose, move joint to the target value
        assert len(joint_target) == self.num_controllable_joint
        for i, joint_id in enumerate(self.controllable_joint_ids):
            p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, joint_target[i], maxVelocity=self.maxvel)
            
            # <<< control mimic joints
            if self.joint_names[self.joint_ids.index(joint_id)] in self.mimic:
                for mimic_name, mimic_ratio in self.mimic[self.joint_names[self.joint_ids.index(joint_id)]].items():
                    mimic_id = self.joint_ids[self.joint_names.index(mimic_name)]
                    p.setJointMotorControl2(self.robot_id, mimic_id, p.POSITION_CONTROL, joint_target[i] * mimic_ratio, maxVelocity=self.maxvel)
    
    def move_delta_joint(self, delta_joint_target):
        # <<< input delta joint (minimum movement), move joint by the delta value
        assert len(delta_joint_target) == self.num_controllable_joint
        joint_state = self.get_joint_pose()
        joint_target = copy.copy(joint_state)
        for i in range(len(self.num_controllable_joint)):
            joint_target[i] += delta_joint_target[i]
        self.move_joint(joint_target)
    
    def move_default_pose(self, pose_name):
        if pose_name not in self.default_pose:
            return False
        js_target = self.default_pose[pose_name]
        return self.move_joint(js_target)
    
    def receive_cmd(self, cmd):
        if "type" not in cmd or "act" not in cmd:
            rospy.logwarn("NOT A CORRECT CMD, REJECT")
            return False
        else:
            self.cmd_queue.append(cmd)
            # rospy.loginfo(f"CMD RECEIVED. {self.struct_name}:  {cmd}")
            return True
    def check_current_cmd(self):
        if len(self.cmd_queue) <=0:
            return None
        else:
            return copy.deepcopy(self.cmd_queue[0])
    def execute_cmd(self):
        if len(self.cmd_queue) == 0:
            return False
        self.count_execute_cmd += 1
        cmd_to_execute =self.cmd_queue[0]
        if self.count_execute_cmd >= 1:
            check = self.get_action_and_execute(cmd_to_execute)
        else:
            check = True
        
        self.check_blocking_mode()
        return check
    
    def get_action_and_execute(self, cmd_to_execute):
        if cmd_to_execute["type"] == "js":
            act = cmd_to_execute["act"]
            # rospy.loginfo(f"EXECUTING ACTION. {self.struct_name}:  {act}")
            self.move_joint(act)
            return True
        elif cmd_to_execute["type"] == "d_pose":
            pose_name = cmd_to_execute["act"]
            self.move_default_pose(pose_name)
            return True
        else:
            cmd_type = cmd_to_execute["type"]
            rospy.logwarn(f"STRUCT {self.struct_name} DOSE NOT SUPPORT CMD TYPE :{cmd_type}")
            return False
    
    def check_blocking_mode(self):
        if not self.blocking_mode:
            self.cmd_queue.pop(0)
            self.count_execute_cmd = 0
        else:
            # <<< not in blocking_mode: cmd will not be poped from cmd_queue until the execution is finished(struct finished moving )
            self.prev_js_pose = copy.deepcopy(self.curr_js_pose)
            self.curr_js_pose = self.get_joint_pose()
            delta_joint_value = np.linalg.norm(self.curr_js_pose - self.prev_js_pose)
            if self.count_execute_cmd >=4 and delta_joint_value < self.block_threshold:
                # action movement stopped, release blocking_mode
                self.cmd_queue.pop(0)
                self.count_execute_cmd = 0
                if self.vis:
                    rospy.loginfo(f"{self.struct_name}  MOVE FINISHED , ")
                    rospy.loginfo(f"current_JS: {self.curr_js_pose}  ")
                    rospy.loginfo(f"speed(joint_value):   {delta_joint_value}")
            elif self.count_execute_cmd > self.fail_step_threshold:
                # takes to long to execute the cmd, treate is as a failure execution, release blocking_mode
                act = self.cmd_queue[0]["act"]
                rospy.logwarn(f"IT TAKES TO LONG TIME TO EXECUTING ACTION. {self.struct_name}:  {act}, STOP EXECUTION")
                self.cmd_queue.pop(0)
                self.count_execute_cmd = 0
        return True
    
    def check_cmd_achievement(self, cmd):
        if "type" not in cmd or "act" not in cmd:
            rospy.logwarn("NOT A CORRECT CMD, REJECT")
            return False, {}
        curr_js = self.get_joint_pose()
        if cmd["type"] == "d_pose":
            target = js_target = self.default_pose[cmd["act"]]
        else:
            target = np.array(cmd["act"])
        
        delta_value = np.linalg.norm(curr_js - target)
                
        check = delta_value < self.achievement_threshold
        return check, {"delta_js":delta_value}
    
    def check_reach(self, target_pos, target_orn, xyz_threshold = None, rpy_threshold = None):
        curr_pos, curr_orn = self.get_end_link_pose_self()
        target_rpy = np.array(p.getEulerFromQuaternion(target_orn))
        curr_rpy = np.array(p.getEulerFromQuaternion(curr_orn))
        
        delta_xyz = np.linalg.norm(curr_pos - target_pos)
        delta_rpy = np.linalg.norm(curr_rpy - target_rpy)
        if xyz_threshold is None:
            xyz_threshold = self.achievement_threshold 
        if rpy_threshold is None:
            rpy_threshold = self.achievement_threshold 
        check_achievement = (delta_xyz < xyz_threshold) and (delta_rpy < rpy_threshold)
        return check_achievement, delta_xyz, delta_rpy
            
        
            
    

class Arm(BaseStruct):
    def __init__(self, config,blocking_mode = True, vis = True):
        super(Arm, self).__init__(config, blocking_mode= blocking_mode, vis = vis)
        
        self.joint_names = self.robot_joints_info["names"]
        self.ll_robot = [0] * len(self.joint_names)
        self.ul_robot = [0] * len(self.joint_names)
        self.jr_robot = [0] * len(self.joint_names)
        for i, name in enumerate(self.joint_names):
            self.ll_robot[i] = self.robot_joints_info["lower_limits"][name]
            self.ul_robot[i] = self.robot_joints_info["upper_limits"][name]
            self.jr_robot[i] = self.robot_joints_info["joint_ranges"][name]
        
    
    def get_action_and_execute(self, cmd_to_execute):
        if cmd_to_execute["type"] == "js":
            act = cmd_to_execute["act"]
            # rospy.loginfo(f"EXECUTING ACTION. {self.struct_name}:  {act}")
            self.move_joint(act)
            return True
        elif cmd_to_execute["type"] == "ee":
            act = cmd_to_execute["act"]
            # rospy.loginfo(f"EXECUTING ACTION. {self.struct_name}:  {act}")
            pos, orn = act[:3], act[3:]
            self.move_end_link_self(pos, orn)
            return True
        elif cmd_to_execute["type"] == "ee_rpy":
            act = cmd_to_execute["act"]
            # rospy.loginfo(f"EXECUTING ACTION. {self.struct_name}:  {act}")
            pos, rpy = act[:3], act[3:]
            self.move_end_link_self(pos, rpy, use_rpy=True)
            return True
        elif cmd_to_execute["type"] == "d_pose":
            pose_name = cmd_to_execute["act"]
            self.move_default_pose(pose_name)
        else:
            cmd_type = cmd_to_execute["type"]
            rospy.logwarn(f"STRUCT {self.struct_name} DOSE NOT SUPPORT CMD TYPE :{cmd_type}")
            return False
    
    def create_fake_js(self, js = None):
        robot_ids = self.robot_joints_info["ids"]
        curr_js_fake = len(robot_ids) * [0]
        for idx, robo_id in enumerate(robot_ids):
            curr_js_fake[idx] = p.getJointState(self.robot_id, robo_id)[0]
        
        if js is not None:
            assert len(js) == len(self.joint_index)
            count_fake = 0
            for idx in self.joint_index:
                curr_js_fake[idx] = js[count_fake]
                count_fake += 1    
        
        return curr_js_fake    
    
    def move_end_link_world(self, target_pos, target_orn, use_rpy = False):
        # input target pose in <<< WORLD COORDINATE >>> move end_link to the target pose in <<< WORLD COORDINATE >>>
        assert len(target_pos) == 3
        if use_rpy:
            assert len(target_orn) == 3
            target_orn = p.getQuaternionFromEuler(target_orn)
        else:
            assert len(target_orn) == 4
            
            
        ###### <<< new IK
        
        curr_js_fake = self.create_fake_js()
        joint_poses = p.calculateInverseKinematics(self.robot_id, self.end_link_id, target_pos, target_orn,
                                                    self.ll_robot, self.ul_robot, self.jr_robot, curr_js_fake,
                                                    maxNumIterations=1000, residualThreshold=1e-4, solver = p.IK_DLS) 
        controllable_joint_poses = [joint_poses[idx] for idx in self.joint_index]
        self.move_joint(controllable_joint_poses)
    
    def move_delta_end_link_world(self, delta_pos, delta_rpy):
        curr_pos, curr_orn = self.get_end_link_pose()
        curr_rpy = p.getEulerFromQuaternion(curr_orn)
        target_pos, target_rpy = copy.copy(curr_pos), copy.copy(curr_rpy)
        for i in range(3):
            target_pos[i] += delta_pos[i]
            target_rpy[i] += delta_rpy[i]
        self.move_end_link_world(target_pos, target_rpy, use_rpy=True)
    
    def move_end_link_self(self, target_pos_self, target_orn_self, use_rpy = False):
        # input target pose in <<< WORLD COORDINATE >>> move end_link to the target pose in <<< ROBOT COORDINATE >>>
        assert len(target_pos_self) == 3
        if use_rpy:
            assert len(target_orn_self) == 3
            target_orn_self = p.getQuaternionFromEuler(target_orn_self)
        else:
            assert len(target_orn_self) == 4
        
        target_pos, target_orn = robot_to_world(self.robot_id, target_pos_self, target_orn_self)    
        
        ###### <<< new IK
        curr_js_fake = self.create_fake_js()
        joint_poses = p.calculateInverseKinematics(self.robot_id, self.end_link_id, target_pos, target_orn,
                                                    self.ll_robot, self.ul_robot, self.jr_robot, curr_js_fake,
                                                    maxNumIterations=2500, residualThreshold=1e-5, solver = p.IK_DLS) 
        
        controllable_joint_poses = [joint_poses[idx] for idx in self.joint_index]
        self.move_joint(controllable_joint_poses)
    
    def move_delta_end_link_self(self, delta_pos, delta_rpy):
        curr_pos, curr_orn = self.get_end_link_pose()
        curr_pos_self, curr_orn_self = world_to_robot(self.robot_id, curr_pos, curr_orn)
        curr_rpy_self = p.getEulerFromQuaternion(curr_orn_self)
        target_pos_self, target_rpy_self = copy.copy(curr_pos_self), copy.copy(curr_rpy_self)
        for i in range(3):
            target_pos_self[i] += delta_pos[i]
            target_rpy_self[i] += delta_rpy[i]
        self.move_end_link_self(target_pos_self, target_rpy_self, use_rpy=True)
    
    def check_cmd_achievement(self, cmd):
        if "type" not in cmd or "act" not in cmd:
            rospy.logwarn("NOT A CORRECT CMD, REJECT")
            return False, {}
        
        if cmd["type"] in["ee", "ee_rpy"]:
            curr_pos, curr_orn = self.get_end_link_pose_self()
            target_pos = np.array(cmd["act"][:3])
            if cmd["type"] == "ee_rpy":
                target_orn = np.array(p.getQuaternionFromEuler(cmd["act"][3:]))
            else:
                target_orn = np.array(cmd["act"][3:])
            delta_xyz = np.linalg.norm(curr_pos - target_pos)
            delta_orn = np.linalg.norm(curr_orn - target_orn)
            check = delta_xyz < self.achievement_threshold and delta_orn < self.achievement_threshold
            return check, {"delta_xyz":delta_xyz, "delta_orn":delta_orn}
        if cmd["type"] == "d_pose":
            target = self.default_pose[cmd["act"]]
        elif cmd["type"] == "js":
            target = np.array(cmd["act"])
        else:
            rospy.logwarn("NOT A CORRECT CMD, REJECT")
            return False, {}
        curr_js = self.get_joint_pose()
        delta_value = np.linalg.norm(curr_js - target)
        check = delta_value < self.achievement_threshold
        return check, {"delta_js":delta_value}
    
    def reset_by_ee_pose_self(self, pos, orn, js = None):

        
        pos_world, orn_world = robot_to_world(self.robot_id, pos, orn)
        
        
        curr_js_fake = self.create_fake_js(js = js)
        joint_poses = p.calculateInverseKinematics(self.robot_id, self.end_link_id, pos_world, orn,
                                                    self.ll_robot, self.ul_robot, self.jr_robot, curr_js_fake,
                                                    maxNumIterations=2500, residualThreshold=1e-4, solver = p.IK_DLS) 
        
        controllable_joint_poses = [joint_poses[idx] for idx in self.joint_index]
        
        self.reset_by_joint_states(controllable_joint_poses)
        self.move_joint(controllable_joint_poses)
        return True

    def solve_robust_ik(self, target_pos, target_orn, seed_js=None):
        """
        增强版 IK 求解器：
        1. 使用 Null Space 计算 (如果参数可用)。
        2. 包含重试机制 (Random Restarts)。
        3. 内部执行 reset 并返回是否成功的标志。
        """
        max_attempts = 3 # 最大重试次数
        best_js = None
        min_error = float('inf')
        
        # 准备 IK 参数
        ll = getattr(self, 'll_robot', None) # Lower limits
        ul = getattr(self, 'ul_robot', None) # Upper limits
        jr = getattr(self, 'jr_robot', None) # Joint ranges
        rp = getattr(self, 'rp_robot', None) # Rest poses
        
        # 如果没有 rest poses，使用 seed_js 作为 rest pose 的一种替代策略，或者保持 None
        if rp is None and seed_js is not None:
             # 注意：rp 长度必须匹配全机器人自由度，这里需要小心处理
             # 假设 self.rp_robot 应该在初始化时被正确设置。如果没有，Null Space 计算会退化为普通 IK。
             pass

        curr_js_fake = self.create_fake_js(js=seed_js) if seed_js is not None else None

        for attempt in range(max_attempts):
            # 转换坐标系
            pos_world, orn_world = robot_to_world(self.robot_id, target_pos, target_orn)
            
            # 如果是重试 (attempt > 0)，给 seed 加一点随机噪声，帮助跳出局部极小值
            if attempt > 0 and curr_js_fake is not None:
                noise = np.random.normal(0, 0.05, size=len(curr_js_fake))
                iter_js = list(np.array(curr_js_fake) + noise)
            else:
                iter_js = curr_js_fake

            # 调用 PyBullet IK
            if ll is not None and ul is not None and jr is not None and rp is not None:
                # 使用 Null Space (最推荐，最稳定)
                joint_poses = p.calculateInverseKinematics(
                    self.robot_id, self.end_link_id, pos_world, target_orn, # 注意：calculateIK 通常接受目标的世界坐标系位置和方向
                    lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp,
                    maxNumIterations=5000, residualThreshold=1e-4, solver=p.IK_DLS
                )
            else:
                # 普通 IK (DLS)
                joint_poses = p.calculateInverseKinematics(
                    self.robot_id, self.end_link_id, pos_world, target_orn,
                    maxNumIterations=5000, residualThreshold=1e-4, solver=p.IK_DLS
                )
            
            # 提取控制关节
            controllable_joint_poses = [joint_poses[idx] for idx in self.joint_index]
            
            # 验证解的质量 (计算 FK 看看是不是真的到了)
            # 临时设置模型状态来检查误差 (不渲染)
            self.reset_by_joint_states(controllable_joint_poses)
            
            curr_pos, curr_orn = self.get_end_link_pose_self()
            # 简单的欧氏距离检查
            err = np.linalg.norm(np.array(curr_pos) - np.array(target_pos))
            
            if err < min_error:
                min_error = err
                best_js = controllable_joint_poses
            
            if err < 0.01: # 阈值可调
                return True, best_js
        
        # 如果重试多次后误差依然很大
        if min_error > 0.02:
            return False, best_js
        
        # 勉强接受
        return True, best_js     

class Head(BaseStruct):
    def __init__(self, config, blocking_mode = False, vis = True):
        super(Head, self).__init__(config, blocking_mode=blocking_mode, vis = vis)
       


class Hand(BaseStruct):
    def __init__(self, config, blocking_mode = False, vis = True):
        super(Hand, self).__init__(config, blocking_mode = blocking_mode, vis = vis) 
    

class Gripper(BaseStruct):
    def __init__(self, config, blocking_mode = False, vis = True):
        super(Gripper, self).__init__(config, blocking_mode=blocking_mode, vis = vis)
        
        

TYPE_MAPPER = {
    "arm":Arm,
    "hand":Hand,
    "gripper":Gripper,
    "head":Head
}





