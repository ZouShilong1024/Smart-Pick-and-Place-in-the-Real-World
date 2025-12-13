import rospy
import tf2_ros
import os, sys
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import tf

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
grand_parent_dir = os.path.dirname(parent_dir)

sys.path.append(grand_parent_dir)
sys.path.append(os.path.dirname(grand_parent_dir))
sys.path.append(parent_dir)

class TransformationUtil():
    def __init__(self):
        super().__init__()
        rospy.init_node("transformation_util")

    def get_transform_from_frame_to_frame(self, from_frame, to_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))

        retries = 5
        backoff_factor = 0.5

        for i in range(retries):
            try:
                (position, quaternion) = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))#在TF变换中查找变换，position是平移，quaternion是旋转
                translation_matrix = np.eye(4,4)#创建单位矩阵
                # position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                # quaternion = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
                orientaion = R.from_quat(quaternion).as_matrix()#得到3*3的旋转矩阵，表示两个坐标系间的旋转关系
                euler = R.from_quat(quaternion).as_euler('xyz', degrees=False)#得到欧拉角
                translation_matrix[:3, 3] = position
                translation_matrix[:3, :3] = orientaion
                transformation_euler = [position[0], position[1], position[2], euler[0], euler[1], euler[2]]#生成欧拉角格式
                transformation_quat = position + quaternion#生成四元数格式
                return translation_matrix, transformation_euler, transformation_quat
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                time.sleep(backoff_factor * (i + 1))

if __name__ == '__main__':
    from_frame = "base_link"
    to_frame = "base_link_r"

    transform = TransformationUtil()
    translation_matrix, transformation_euler, transformation_quat = transform.get_transform_from_frame_to_frame(from_frame, to_frame)
    print(transformation_euler)