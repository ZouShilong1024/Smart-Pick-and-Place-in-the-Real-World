import socket
import json
import time, struct
from termcolor import cprint

# 连接到运行 HandController 的机器 IP
HOST = '127.0.0.1' 
PORT = 8020

def send_cmd(sock, data):
    msg = json.dumps(data).encode('utf-8')
    length_prefix = struct.pack('>I', len(msg))
    sock.sendall(length_prefix)
    sock.sendall(msg)
    resp = json.loads(sock.recv(1024).decode('utf-8'))
    cprint(f"Response: {resp}", "red")

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))

try:
    print("Sending Close command...")
    data = {
        "srv": "twin_inference",
        "type": "trajectory_generation2",
        "cnfg":{
            "target_pose":[
                # [-0.28701299065166197, -0.10952208102805991, 0.18770306518519692, 0.7429832323953516, -0.3406782645617565, -0.5639482092149282, -0.11779920949573683],
                [-0.29824502615690757, -0.1048591177826923, 0.1558604879639906, 0.7429832323953516, -0.3406782645617565, -0.5639482092149282, -0.11779920949573683]
                ],
            "current_js": [-0.021013764194011724, -0.15385077356330015, 1.0034596001416198, -0.05811946409141117, 1.986167235477027, -0.00020943951023931956],
            "struct": "left_arm"
        }
    }
    send_cmd(client, data)
    
finally:
    client.close()
