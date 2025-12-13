#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy, sys, struct
import socket
from scipy.spatial.transform import Rotation as R
import os, sys, json
from termcolor import cprint

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
grand_parent_dir = os.path.dirname(parent_dir)
sys.path.append(parent_dir)
sys.path.append(grand_parent_dir)
sys.path.append(os.path.dirname(grand_parent_dir))

def send_cmd(sock, data):
    data_bytes = json.dumps(data).encode('utf-8')
    length_prefix = struct.pack('>I', len(data_bytes))
    sock.sendall(length_prefix)
    sock.sendall(data_bytes)
    resp = json.loads(sock.recv(1024).decode('utf-8'))
    cprint(f"Control arm response: {resp}", "red")
    return resp

class ArmController():
    def __init__(self):
        super().__init__()
        self.cmds_arm = []

    def reset_cmd(self,):
        self.cmds_arm = []
    
    def start_cmd(self,):
        self.cmds_arm.append({"type": "start", "act": []})

    def add_ee_cmd(self, execute_traj_ee, speed=5, block=True):
        self.cmds_arm.append({"type": "ee", "act": execute_traj_ee, "speed": speed, "block": block})

    def add_js_cmd(self, execute_traj_js, speed=5, block=True):
        self.cmds_arm.append({"type": "js", "act": execute_traj_js, "speed": speed, "block": block})

    def send_cmds(self, client):
        self.cmds_arm.append({"type": "end", "act": []})
        req = {"srv": "/right_arm/movement_control", "cmd": self.cmds_arm}
        resp = send_cmd(client, req)
        state = resp["value"]
        info = resp["info"] if "info" in resp else {}
        return True

if __name__ == "__main__":
    service_name = "/right_arm/movement_control"
    controller = ArmController(service_name=service_name)