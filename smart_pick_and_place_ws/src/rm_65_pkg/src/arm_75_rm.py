#!/home/zz/anaconda3/envs/anygrasp/bin/python3
from Robotic_Arm.rm_robot_interface import *
import copy
import numpy as np
import rospy

def generate_consecutive_arrays(a1, an_plus_1, n):
    # Calculate the total difference between a(n+1) and a1
    total_diff = np.array(an_plus_1) - np.array(a1)
    
    # The differences must follow the rule: (a3-a2) = (a2-a1)*2, and so on.
    # So we need to reverse engineer the differences.
    diffs = [total_diff / (2 ** (n - 1))]  # Start with the last difference as the base
    
    # Generate the differences by dividing by 2 each time
    for i in range(n - 1, 0, -1):
        diffs.insert(0, diffs[0] * 2)  # Double the previous difference
    
    # Now generate the arrays
    result = [a1]
    for i in range(n):
        # For each step, add the current difference to the last array
        result.append(result[-1] + diffs[i])
    
    return result


class RM_ARM():
    def __init__(self, arm_ip = None):
        self.arm_id = -1
        self.arm_ip = arm_ip
        self.arm_port = 8080
        self.arm_type = "RM_75B"
        self.arm_TCP = "end_effector"
        self.arm_workcoordinate = "base_link"
        # arm move arg config
        self.arm_speed = 0.1 #rm_set_arm_max_line_speed
        self.arm_acc = 0.01
        self.arm_angular_speed = 1
        self.arm_angular_acc = 0.01

        self.movejoint_v = 15  #joint speed rate 10%
        self.movejoint_r = 50  #交融半径百分比系数，0~100
        self.movejoint_connect = 0
        self.movejoint_block = 0 #0 no_block_mode 1 block_mode


    def arm_connet(self, mode=None):  #connet to the arm
        if mode is not None:
            thread_mode = rm_thread_mode_e.RM_DUAL_MODE_E
            self.robot_arm = RoboticArm(thread_mode)
        else:
            self.robot_arm = RoboticArm()
        #self.robot_arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        self.handle = self.robot_arm.rm_create_robot_arm(self.arm_ip, self.arm_port, level=3)
        print(self.handle.id)
        self.arm_id = self.handle.id
        if self.arm_id == -1:
            print("Faild to connet the robot arm!")
        else:
            print("Succeed to connnet the robot arm!")
        return self.handle.id
    
    def set_arm_config(self, arm_id = None, arm_ip = "192.168.1.18", arm_port = 8080, arm_type = "RM_75B", arm_TCP = None, arm_workcoordinate = None):
        self.arm_id = arm_id
        self.arm_ip = arm_ip
        self.arm_port = arm_port
        self.arm_type = arm_type
        self.arm_TCP = arm_TCP
        self.arm_workcoordinate = arm_workcoordinate
        #set arm move config
        #set arm max line speed
        arm_speed_tag = self.robot_arm.rm_set_arm_max_line_speed(self.arm_speed)
        if arm_speed_tag == 0:
            print("succeed to set arm max line speed!")
        else:
            print(f"faild to set arm max line speed!\nthe set arm speed tag is {arm_speed_tag}")
        #set arm max line acc
        arm_acc_tag = self.robot_arm.rm_set_arm_max_line_acc(self.arm_acc)
        if arm_acc_tag == 0:
            print("succeed to set arm max line acc speed!")
        else:
            print(f"faild to set arm max line acc speed!\nthe set arm speed acc tag is {arm_acc_tag}")
        arm_angular_speed_tag = self.robot_arm.rm_set_arm_max_angular_speed(self.arm_angular_speed)
        if arm_angular_speed_tag == 0:
            print("succeed to set arm max angular speed!")
        else:
            print(f"faild to set arm max line speed!\nthe set arm angular tag is {arm_angular_speed_tag}")
        arm_angular_acc_tag = self.robot_arm.rm_set_arm_max_angular_acc(self.arm_angular_acc)
        if arm_angular_acc_tag == 0:
            print("succeed to set arm max angular speed!")
        else:
            print(f"faild to set arm max angular speed!\nthe set arm angular tag is {arm_angular_acc_tag}")
         
    def read_arm_config(self):
        arm_id  = self.arm_id
        arm_ip = self.arm_ip
        arm_port = self.arm_port
        arm_type = self.arm_type
        arm_TCP = self.arm_TCP
        arm_workcoordinate = self.arm_workcoordinate
        print(f"arm type:{arm_type}\narm id:{arm_id}\narm ip:{arm_ip}\narm port:{arm_port}\narm TCP:{arm_TCP}\narm workcoordinate:{arm_workcoordinate}\n")
        #get arm move config
        tag, arm_max_line_speed = self.robot_arm.rm_get_arm_max_line_speed()
        if tag == 0:
            print(f"succeed to get arm max line speed!\nthe arm max line speed is {arm_max_line_speed}")
        else:
            print(f"faild to get arm max line speed!\nthe get arm line speed tag is {tag}")
        tag, arm_max_line_acc = self.robot_arm.rm_get_arm_max_line_acc()
        if tag == 0:
            print(f"succeed to get arm max line acc!\nthe arm max line acc is {arm_max_line_acc}")
        else:
            print(f"faild to get arm max line acc!\nthe get arm max line acc tag is {tag}")
        tag, arm_max_angular_speed = self.robot_arm.rm_get_arm_max_angular_speed()
        if tag == 0:
            print(f"succeed to get arm max angular speed!\nthe arm max angular speed is {arm_max_angular_speed}")
        else:
            print(f"faild to get arm max angular speed!\nthe get arm angular tag is {tag}")
        tag, arm_max_angular_acc = self.robot_arm.rm_get_arm_max_angular_acc()
        if tag == 0:
            print(f"succeed to get arm max angular acc!\nthe arm max angular acc is {arm_max_angular_acc}")
        else:
            print(f"faild to get arm max angular acc!\nthe get arm angular acc tag is {tag}")        

    def read_joint_state(self):
        #tag, arm_states = self.robot_arm.rm_get_arm_all_state()
        tag, arm_states = self.robot_arm.rm_get_current_arm_state()
        if tag == 0:
            print("Succeed to get current joint state and the end pose!")
            joint_current_state = arm_states["joint"]
            pose_current = arm_states["pose"]
            print(joint_current_state)
        else:
            print("Failed to get current joint state and the end pose!")
        return joint_current_state, pose_current

    def move_joint_trajectory(self, end_joint, connect_tag, movejoint_speed, move_joint_block):
        # print(end_joint)
        tag = self.robot_arm.rm_movej(joint=end_joint, v=movejoint_speed, r=self.movejoint_r, connect=connect_tag, block=move_joint_block)
        if tag == 0:
            log = "Succeed to move joint to end_joint!"
            print("Succeed to move joint to end_joint!")
        else:
            log = "Faild to move joint to end_joint!\nthe rm_movej tag is {}".format(tag)
            print(f"Faild to move joint to end_joint!\nthe rm_movej tag is {tag}")
        return log

    def move_pose_trajectory(self, end_pose, connect_tag, movepose_speed, movepose_block):
        tag = self.robot_arm.rm_movel(pose=end_pose, v=movepose_speed, r=self.movejoint_r, connect=connect_tag, block=movepose_block)
        if tag == 0:
            log = "Succeed to move joint to end_pose!"
            print("Succeed to move joint to end_pose!")
        else:
            log = "Faild to move joint to end_pose!\nthe rm_movep tag is {}".format(tag)
            print(f"Faild to move joint to end_pose!\nthe rm_movep tag is {tag}")
        return log

    def generate_joint_follow_trajectory(self, trajectory, current_js, delta_threshold = 1.0, mb = 0):
        joint_follow_traj = []
        joint_follow_traj.append(np.array(current_js).reshape(1, -1))

        start_js = np.array(current_js)
        for i, js in enumerate(trajectory):
            # if i + 4 > len(trajectory) and mb:
            #     delta_threshold /= 1.3
            # if i < 3 :
            #     delta_threshold *= 1.3
            target_js = np.array(js)
            delta_js = target_js - start_js
            num_point = int(np.linalg.norm(delta_js)/delta_threshold)
            
            # if i < 1:
            #     static_num = 100
            #     soft_start_num = 100
            #     traj_fragment = np.zeros((num_point + soft_start_num + static_num, 7))
            #     first_js = start_js[:] + delta_js * (i+1) / num_point
            #     soft_start_ps = generate_consecutive_arrays(start_js, first_js, soft_start_num)
            #     for i in range(static_num):
            #         traj_fragment[i, :] = start_js[:]
            #     for i in range(soft_start_num):
            #         traj_fragment[i + static_num, :] = soft_start_ps[i][:]
            #     for i in range(num_point):
            #         traj_fragment[i+soft_start_num + static_num, :] = start_js[:] + delta_js * (i+1) / num_point
            # elif i >= len(trajectory) - 1:
            #     static_num = 100
            #     soft_end_num = 100
            #     traj_fragment = np.zeros((num_point + soft_end_num + static_num, 7))
            #     for i in range(num_point - 1):
            #         traj_fragment[i, :] = start_js[:] + delta_js * (i+1) / num_point
            #     last_second_js = start_js[:] + delta_js * (num_point - 1) / num_point
            #     last_js = start_js[:] + delta_js * 1
            #     soft_end_ps = generate_consecutive_arrays(last_second_js, last_js, soft_end_num)
                
            #     for i in range(soft_end_num):
            #         traj_fragment[i + num_point - 1, :] = soft_end_ps[i][:]
            #     for i in range(static_num):
            #         traj_fragment[i + num_point + soft_end_num - 1, :] = last_js[:]
                
            # else:
            #     traj_fragment = np.zeros((num_point, 7))
            #     for i in range(num_point):
            #         traj_fragment[i, :] = start_js[:] + delta_js * (i+1) / num_point
            

            traj_fragment = np.zeros((num_point, 7))
            # first_js = start_js[:] + delta_js * (i+1) / num_point
            # soft_start_ps = generate_consecutive_arrays(start_js, first_js, soft_start_num)

            for i in range(num_point):
                traj_fragment[i, :] = start_js[:] + delta_js * (i+1) / num_point


            joint_follow_traj.append(traj_fragment)
            
            rospy.logwarn(f"--------------{i}-------{num_point}------{traj_fragment.shape}")
            rospy.logwarn(start_js)
            rospy.logwarn(target_js)
            start_js = copy.copy(js)

        joint_follow_traj = np.vstack(joint_follow_traj)
        shape = joint_follow_traj.shape
        rospy.logwarn(f"JOINT FOLLOW TRAJECTORY CREATED, SHAPE : [{shape[0]}, {shape[1]}]")
        return joint_follow_traj


    def move_joint_follow(self, trajectory, current_js, interval_time = 0.013, mb = 0):
        joint_follow_traj = self.generate_joint_follow_trajectory(trajectory, current_js, mb = mb)
        interval_time = max(0.01, interval_time)
        
        for j in range(joint_follow_traj.shape[0]):
            tag = self.robot_arm.rm_movej_canfd(joint= list(joint_follow_traj[j,:]), follow=True, expand=0)
            rospy.sleep(interval_time)
        
        if tag == 0:
            print("Succeed to move end effector to end pose!")
        else:
            print(f"Faild to move end effector to end pose!\nthe rm_movel tag is {tag}")
        
        return tag


if __name__ == "__main__":
    robot_ip = "192.168.1.19"
    arm = RM_ARM(arm_ip=robot_ip)
    arm_id = arm.arm_connet(mode=1)
    arm.set_arm_config(arm_id=arm_id, arm_ip=robot_ip, arm_port=8080, arm_type="RM_75B", arm_TCP="end_effector", arm_workcoordinate="base_link")
    arm.read_arm_config()
    arm.read_joint_state()

    # arm.move_joint_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],0,20,0)
    # arm2.move_joint_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])