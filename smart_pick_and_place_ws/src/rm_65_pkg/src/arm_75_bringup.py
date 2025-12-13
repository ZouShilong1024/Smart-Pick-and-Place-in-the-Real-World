#!/home/zz/anaconda3/envs/anygrasp/bin/python3
import os
import sys
import socket
from Robotic_Arm.rm_robot_interface import *
from arm_75_rm import RM_ARM
import rospy
from std_msgs.msg import String
import json, time
from sensor_msgs.msg import JointState

import numpy as np
import threading
import struct

def degree_2_pi(pose_degree):
    assert len(pose_degree) == 7
    return [j*np.pi/180 for j in pose_degree]

def pi_2_degree(pose_pi):
    assert len(pose_pi) == 7
    return [j*180/np.pi for j in pose_pi]


def rm_75_extract_traj(data):
        js_trajectory = []
        js_type = []
        js_moveit_traj_connect_tag = []
        js_speed = []
        js_move_joint_block = []
        ee_trajectory = []
        ee_type = []
        ee_moveit_traj_connect_tag = []
        ee_speed = []
        ee_move_joint_block = []
        data_dict = json.loads(data)
        for data in data_dict["cmd"]:
            if data["type"] == "start":
                continue
            elif data["type"] == "ee":
                traj = [data["act"]['x'], data["act"]['y'], data["act"]['z'], data["act"]['rx'], data["act"]['ry'], data["act"]['rz']]
                ee_trajectory.append(traj)
                ee_type.append(data["type"])
                ee_connect_tag = 1
                ee_moveit_traj_connect_tag.append(ee_connect_tag)
                ee_speed.append(data["speed"])
                ee_move_joint_block.append(int(data["block"]))
            elif data["type"] == "js":
                traj = [data["act"]['J1'], data["act"]['J2'], data["act"]['J3'], data["act"]['J4'], data["act"]['J5'], data["act"]['J6'], data["act"]['J7']]
                js_trajectory.append(traj)
                js_type.append(data["type"])
                js_connect_tag = 1
                js_moveit_traj_connect_tag.append(js_connect_tag)
                js_speed.append(data["speed"])
                js_move_joint_block.append(int(data["block"]))
            elif data["type"] == "get_force":
                pass
            elif data["type"] == "end":
                print("*"*80)
                print("len js moveit traj connect tag: ", len(js_moveit_traj_connect_tag))
                print("len ee moveit traj connect tag: ", len(ee_moveit_traj_connect_tag))
                if(len(js_moveit_traj_connect_tag)) != 0 :
                    js_moveit_traj_connect_tag[len(js_moveit_traj_connect_tag) - 1] = 0
                if(len(ee_moveit_traj_connect_tag)) != 0 :
                    ee_moveit_traj_connect_tag[len(ee_moveit_traj_connect_tag) - 1] = 0
        print("--"*10)
        print("js moveit_traj_connect_tag",js_moveit_traj_connect_tag)
        print("ee moveit_traj_connect_tag",ee_moveit_traj_connect_tag)
        print("js trajectory: ", js_trajectory)
        print("ee trajectory: ", ee_trajectory)
        return js_type, js_trajectory, js_moveit_traj_connect_tag, js_speed, js_move_joint_block, ee_type, ee_trajectory, ee_moveit_traj_connect_tag, ee_speed, ee_move_joint_block

class RM_ARM_bringup(RM_ARM):
    def __init__(self, arm_ip):
        super().__init__(arm_ip)
        rospy.init_node('arm_65_bringup', anonymous=True)
        self.publish_rate = 20

        self.server_ip = '0.0.0.0'
        self.server_port = 8010
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(5)

        self.socket_thread = threading.Thread(target=self.socket_server_worker)
        self.socket_thread.daemon = True 
        self.socket_thread.start()

    def socket_server_worker(self):
        """Socket 监听循环"""
        while not rospy.is_shutdown():
            try:
                self.server_socket.settimeout(1.0)
                try:
                    conn, addr = self.server_socket.accept()
                    self.handle_client(conn)
                except socket.timeout:
                    continue
            except Exception as e:
                print(f"Socket Server Error: {e}")
                rospy.sleep(1)

    def handle_client(self, conn):
        """处理单个客户端请求"""
        with conn:
            while not rospy.is_shutdown():
                try:
                    length_bytes = b''
                    while len(length_bytes) < 4:
                        chunk = conn.recv(4 - len(length_bytes))
                        if not chunk:
                            raise ConnectionError("Connection closed")
                        length_bytes += chunk
                    data_length = struct.unpack('>I', length_bytes)[0]
                    data_bytes = b''
                    while len(data_bytes) < data_length:
                        chunk = conn.recv(min(4096, data_length - len(data_bytes)))
                        if not chunk:
                            raise ConnectionError("Connection closed")
                        data_bytes += chunk
                    msg_str = data_bytes.decode('utf-8')
                    response = self.rm_75_execute_trajectory_usr(msg_str)
                    conn.sendall(response.encode('utf-8'))
                except Exception as e:
                    print(f"Client handler error: {e}")
                    break

    def publish_joint_state(self):
        pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            _, self.arm_joint_state = self.robot_arm.rm_get_current_arm_state()
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
            joint_state_msg.position = degree_2_pi(self.arm_joint_state["joint"])
            pub.publish(joint_state_msg)
            rate.sleep()

    def rm_75_execute_trajectory_usr(self, data):
        print("=== RECEIVED REQUEST (RIGHT ARM) ===")
        print(f"Data type: {type(data)}")
        print(f"Data content: {data}")
        print("data: ", data)

        js_traj_type, js_trajectory, js_move_connect_tag, js_move_jont_speed ,js_move_joint_block, ee_traj_type, ee_trajectory, ee_move_connect_tag, ee_move_jont_speed ,ee_move_joint_block = rm_75_extract_traj(data)
        rospy.loginfo('EE trajectory received:')
        eetag = 1
        for t in js_trajectory:
            rospy.loginfo(t)
        if len(js_traj_type) > 1:
            log = self.move_joint_trajectory(end_joint=js_trajectory[0], connect_tag=0, movejoint_speed=20, move_joint_block=0)
            is_position = True
            rospy.logwarn(f"-------{is_position}-----davit js")
            while is_position:
                    dif = np.abs(np.array(self.arm_joint_state["joint"]) - np.array(js_trajectory[0]))
                    booldif = np.linalg.norm(dif) < 0.5
                    if np.all(booldif):
                        is_position = False
                        break
            time.sleep(0.1)
            mb = js_move_joint_block[-1]
            rospy.logwarn(f"NOW IN CAN_F MODE, CURRENT POSE IS : {js_trajectory[0]}")
            log = self.move_joint_follow(js_trajectory, js_trajectory[0], mb = mb)
            if mb:
                rospy.sleep(0.5)
            check = True
        
        elif len(js_traj_type) == 1:
            js_end_trajectory = js_trajectory[-1]
            for ty, tr ,mt, mv, mb in zip(js_traj_type, js_trajectory, js_move_connect_tag, js_move_jont_speed, js_move_joint_block):
                if ty == "ee":
                    log = self.move_pose_trajectory(end_pose=tr, connect_tag=mt, movepose_speed=mv, movepose_block=mb)
                    eetag = 0
                elif ty == 'js':
                    log = self.move_joint_trajectory(end_joint=tr, connect_tag=mt, movejoint_speed=mv, move_joint_block=mb)
                else:
                    print("Error: The type is undefinition!")

            if mb == 1 and len(ee_traj_type) == 0:
                is_position = True
                while is_position:
                    dif = np.abs(np.array(self.arm_joint_state["joint"]) - np.array(js_end_trajectory))
                    booldif = dif < 0.05
                    if np.all(booldif):
                        is_position = False
                        break
            check = True

        return_dict = {"srv": "/right_arm/movement_control", "value": check, "info": log}
        rsp = json.dumps(return_dict)
        
        return rsp

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    robot_ip_r = "192.168.1.19"
    arm = RM_ARM_bringup(arm_ip=robot_ip_r)
    arm_id_r = arm.arm_connet(mode=1)
    arm.set_arm_config(arm_id=arm_id_r, arm_ip=robot_ip_r, arm_port=8080, arm_type="RM_75B", arm_TCP="end_effector", arm_workcoordinate="base_link")
    arm.read_arm_config()

    arm.publish_joint_state()
    arm.spin()