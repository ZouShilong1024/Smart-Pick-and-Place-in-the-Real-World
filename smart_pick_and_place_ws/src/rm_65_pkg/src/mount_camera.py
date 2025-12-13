#!/home/zz/anaconda3/envs/anygrasp/bin/python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tr
import numpy as np
import os, sys

class MountCamera():
    def __init__(self, name, parent_frame, child_frame):
        super().__init__()
        node_name = 'static_tf2_broadcaster_' + name
        rospy.init_node(f'{node_name}', anonymous=True)
        self.parent_frame = parent_frame
        self.child_frame = child_frame

    def matrix_to_list(self, matrix_4x4):
        translation = tr.translation_from_matrix(matrix_4x4)
        quaternion = tr.quaternion_from_matrix(matrix_4x4)
        return list(translation) + list(quaternion)

    def calculate_and_broadcast(self):
        # M = [5.9347703553566745e-02, 8.9649222326041933e-03, 8.4151605238849683e-02, 
        #      -5.4467853498316133e-03,  2.8652407786822413e-03, 7.1181678637942480e-01, 7.0314294620378559e-01]
        T_matrix = np.array([
            [-0.06503404712968464, -0.997644867154117, 0.021801187008471494, 0.0708009744931787], 
            [0.9978377577069856, -0.0652237167782832, -0.00810407699381243, 0.023445568410749785], 
            [0.00950694526276967, 0.021227006634725668, 0.9997294794998796, 0.09466674449783057], 
            [0.0, 0.0, 0.0, 1.0]
        ])
        L = self.matrix_to_list(T_matrix)
        # N = [-0.059, -0.000, 0.000,
        #      0.501, -0.498, 0.501, 0.501]
        # N = [0, 0, 0,
        #      0, 0, 0, 1]
        # L = self.calculate_transform(M, N)
        self.publish_static_transform(L, parent_frame, child_frame)
        rospy.spin()

    def calculate_transform(self, M, N):
        translation_M = M[:3]
        quaternion_M = M[3:]
        matrix_M = tr.concatenate_matrices(
            tr.translation_matrix(translation_M),
            tr.quaternion_matrix(quaternion_M)
        )
        translation_N = N[:3]
        quaternion_N = N[3:]
        matrix_N = tr.concatenate_matrices(
            tr.translation_matrix(translation_N),
            tr.quaternion_matrix(quaternion_N)
        )
        matrix_N_inv = tr.inverse_matrix(matrix_N)
        matrix_L = np.dot(matrix_M, matrix_N_inv)
        translation_L = tr.translation_from_matrix(matrix_L)
        quaternion_L = tr.quaternion_from_matrix(matrix_L)

        return list(translation_L) + list(quaternion_L)

    def publish_static_transform(self, L, parent_frame, child_frame):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame
        static_transformStamped.child_frame_id = child_frame
        static_transformStamped.transform.translation.x = L[0]
        static_transformStamped.transform.translation.y = L[1]
        static_transformStamped.transform.translation.z = L[2]
        static_transformStamped.transform.rotation.x = L[3]
        static_transformStamped.transform.rotation.y = L[4]
        static_transformStamped.transform.rotation.z = L[5]
        static_transformStamped.transform.rotation.w = L[6]

        broadcaster.sendTransform(static_transformStamped)
        rospy.loginfo("Published static transform from {} to {}".format(parent_frame, child_frame))

if __name__ == '__main__':
    name = 'realsense'
    parent_frame = 'Link7'
    child_frame = 'cam_link_grasp'
    mc = MountCamera(name, parent_frame, child_frame)
    mc.calculate_and_broadcast()
