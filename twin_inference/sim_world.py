#!/usr/bin/env python3
import os, sys

import rospy
import numpy as np
import json
import time
import pybullet as p
import pybullet_data
import threading
from robot import ErdaijiRobot
import tqdm


SIMULATION_STEP_DELAY = 1 / 240.
ROUND_NUMBER = 3

class World:

    def __init__(self, vis = True, blocking_mode = False):
        
        self.vis = vis
        self.camera_refresh_freq = 10
        self.blocking_mode = blocking_mode
        
        self.robot_path = "/home/zz/ros_proj/erdaiji_ws/src/robot_bringup/easy_dual_arm/urdf/test0/easy_dual_arm.urdf"
        self.robot_config_path = "/home/zz/ros_proj/erdaiji_ws/src/robot_bringup/easy_dual_arm/urdf/test0/robot_config.json"
        self.robot = ErdaijiRobot((0.0, 0.0, 0.0), (0, 0, 0), robot_path = self.robot_path, config_path = self.robot_config_path, blocking_mode=self.blocking_mode, vis=self.vis)
        self.setup()
    
    def setup(self):
        self.num_step = 0
        # <<< setup bullet physics
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        # p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
        # p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
        # p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera( cameraDistance=2.7, cameraYaw=63, cameraPitch=-44, cameraTargetPosition=[0,0,0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        
        # load objects
        self.planeID = p.loadURDF("plane.urdf")
        self.robot.load_robot()
    
    def reset(self):
        self.num_step = 0
        self.robot.reset_robot()
    
    def step(self):
        self.num_step += 1
        self.robot.apply_actions()
        p.stepSimulation()
        rospy.sleep(SIMULATION_STEP_DELAY)
        
        # if self.num_step % self.camera_refresh_freq ==0:
        #     self.robot.update_camera()
        
        robot_state = self.robot.get_state()
        return robot_state
    
    def __del__(self):
        p.disconnect(self.physicsClient)
        



if __name__ == '__main__':
    
    
    rospy.init_node('sim_world', anonymous=True)
    sim_world = World(blocking_mode = True)
    sim_world.reset()
    
    for i in range(10000000):
        # sim_world.reset()
        # sim_world.robot.move_agv([(i/1000)%2,0,0])
        state = sim_world.step()
        # for k, v in state.items():
        #     rospy.loginfo(f"{k}:{v}")
        v = state["ee_self"]["left_arm"]
        rospy.logwarn(f"{v}")
        # rospy.loginfo("==========================")
        # if i % 2000 == 0:
        #     cmd = {"agv":{"type":"pos", "act":[1.0, 1.0, 0.0] }}
        #     sim_world.robot.receive_cmds(cmd)
        #     print("#################3")
        # if i % 2000 == 800:
        #     cmd = {"agv":{"type":"pos", "act":[0.0, 0.0, 0.0] }}
        #     sim_world.robot.receive_cmds(cmd)
        #     print("#################3")
        
        
        if i % 50 == 0:
            cmd = {"right_arm":{"type":"ee", "act":[0.55, -0.2, 0.8, 0, 0.707, 0.0, 0.707] }}
            sim_world.robot.receive_cmds(cmd)
        if i % 50 == 25:
            cmd = {"left_arm":{"type":"ee", "act":[0.45, 0.2, 0.8, 0.707, 0.707, 0.0, 0.0] }}
            sim_world.robot.receive_cmds(cmd)

    #     if i % 50 == 5:
    #         cmd = {"left_gripper":{"type":"js", "act":[1.0- (i//50)%2] }}
    #         sim_world.robot.receive_cmds(cmd)
    #         print("#################3")
        
    #     if i % 100 == 1:
    #         cmd = {"left_arm":{"type":"js", "act":[0.0- (i//100)%2] * 7}}
    #         sim_world.robot.receive_cmds(cmd)
    #         print("#################3")
        
    #     # if i % 200 == 1:
    #     #     cmd = {"right_arm":{"type":"js", "act":[1.0- (i//200)%2] * 7}}
    #     #     sim_world.robot.receive_cmds(cmd)
    #     #     print("#################3")
        
    #     if i % 2 == 1:
    #         cmd = {"head":{"type":"js", "act":[i%360/(360/3.14), 0.2 - i//2%2*0.4]}}
    #         sim_world.robot.receive_cmds(cmd)
    #         print("#################3")