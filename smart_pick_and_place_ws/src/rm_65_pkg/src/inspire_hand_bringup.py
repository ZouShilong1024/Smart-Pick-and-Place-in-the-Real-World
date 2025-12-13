#!/home/zz/anaconda3/envs/anygrasp/bin/python3
import rospy
import json
import os
import sys, struct
import socket
import threading
from sensor_msgs.msg import JointState
from hand_controller_modbus import HandController
import copy

def degree_2_joint_limit(pose_degree):
    assert len(pose_degree) == 6
    pose_degree[0] = 1.7 - pose_degree[0] / 1000 * 1.7
    pose_degree[1] = 0.92 - pose_degree[1] / 1000 * 0.92
    pose_degree[2] = 1.7 - pose_degree[2] / 1000 * 1.7
    pose_degree[3] = 1.7 - pose_degree[3] / 1000 * 1.7
    pose_degree[4] = 1.7 - pose_degree[4] / 1000 * 1.7
    pose_degree[5] = 1.7 - pose_degree[5] / 1000 * 1.7
    return pose_degree

def pi_2_degree(pose_pi):
    assert len(pose_pi) == 6
    return [j*180/3.14 for j in pose_pi]

class Hand_bringup():
    def __init__(self, config_path):
        super().__init__()
        rospy.init_node('hand_bringup', anonymous=True)
        self.config = json.load(open(config_path))
        self.publish_rate = 15
        self.hand_controller = HandController()

        self.server_ip = '0.0.0.0'
        self.server_port = 8000
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
                    data = conn.recv(1024)
                    if not data: break
                    
                    msg_str = data.decode('utf-8')
                    try:
                        req_data = json.loads(msg_str)
                    except json.JSONDecodeError:
                        conn.sendall(json.dumps({"status": "error", "info": "Invalid JSON"}).encode('utf-8'))
                        continue

                    response = self.process_command(req_data)
                    conn.sendall(json.dumps(response).encode('utf-8'))
                except Exception as e:
                    print(f"Client handler error: {e}")
                    break

    def process_command(self, data):
        """
        Socket 指令处理逻辑
        JSON 协议: {"type": "set", "cmd": [1000, 1000, ...]}
        """
        cmd_type = data["type"]
        
        if cmd_type == "set":
            angle_list = data.get("cmd")
            if angle_list and len(angle_list) == 6:
                success = self.hand_controller.write6('angleSet', angle_list)
                return {"srv": "/left_hand/movement_control", "value": True, "info": "Succeed to control hand joint to default joint!"}
            else:
                return {"srv": "/left_hand/movement_control", "value": False, "info": "error"}
        elif cmd_type == "get":
            act_angle = self.hand_controller.read6('angleAct')
            return {"srv": "/left_hand/movement_control", "value": act_angle, "info": "Succeed to get hand joint position!"}
        else:
            return {"srv": "/left_hand/movement_control", "value": False, "info": "Unknown type"}

    def publish_joint_state(self):
        pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.hand_joint_state = self.get_hand_current_state()
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()

            controllable_joint_name = copy.deepcopy(self.config["left_hand"]["controllable_joint_names"])
            position_list = degree_2_joint_limit(self.hand_joint_state)

            data_dict = {}
            for id, name in enumerate(controllable_joint_name):
                data_dict[name] = position_list[id]

            for key in self.config["left_hand"]["mimic"]:
                value_dict = self.config["left_hand"]["mimic"][key]
                for subkey, value in value_dict.items():
                    controllable_joint_name.append(subkey)
                    position_list.append(data_dict[key] * value)

            joint_state_msg.name = controllable_joint_name
            joint_state_msg.position = position_list
            pub.publish(joint_state_msg)
            rate.sleep()
    
    def get_hand_current_state(self):
        state = self.hand_controller.read6('angleSet')
        state = [s / 65535 * 1000  if s >= 65500 else s for s in state]
        pinky_angle, ring_angle, middle_angle, index_angle, thumb_angle, base_angle = state
        return [pinky_angle, ring_angle, middle_angle, index_angle, thumb_angle, base_angle][::-1]
    
    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    config_path = "robot_config.json"
    urdf_dir  = os.path.join(os.path.dirname(__file__), '../../rm_description/urdf/SingleArm')
    hand = Hand_bringup(config_path=os.path.join(urdf_dir, config_path))
    hand.publish_joint_state()