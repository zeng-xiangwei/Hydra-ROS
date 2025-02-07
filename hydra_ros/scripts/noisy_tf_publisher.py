#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


def invert_pose(p):
    p_inv = np.zeros((4, 4))
    p_inv[:3, :3] = p[:3, :3].T
    p_inv[:3, 3] = -p[:3, :3].T @ p[:3, 3]
    p_inv[3, 3] = 1
    return p_inv


def mat_from_pose(pose):

    pose_out = np.zeros((4, 4))
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    t = [pose.position.x, pose.position.y, pose.position.z]
    Rmat = R.from_quat(q).as_matrix()
    pose_out[:3, :3] = Rmat
    pose_out[:3, 3] = t
    pose_out[3, 3] = 1
    return pose_out


def Exp(rho, theta):
    cos = np.cos
    sin = np.sin
    norm = np.linalg.norm
    xi_hat = np.zeros((4, 4))
    theta_hat = np.array(
        [[0, -theta[2], theta[1]], [theta[2], 0, -theta[0]], [-theta[1], theta[0], 0]]
    )
    xi_hat[:3, :3] = theta_hat
    xi_hat[:3, 3] = rho
    return (
        np.eye(4)
        + xi_hat
        + (1 - cos(norm(theta))) / (norm(theta) ** 2) * xi_hat @ xi_hat
        + (norm(theta) - sin(norm(theta)))
        / (norm(theta) ** 3)
        * xi_hat
        @ xi_hat
        @ xi_hat
    )


def calc_odom_delta(odom_old, odom_new):

    w_T_old = mat_from_pose(odom_old.pose.pose)
    w_T_new = mat_from_pose(odom_new.pose.pose)

    delta = invert_pose(w_T_old) @ w_T_new

    return delta


class NoisyTransformPublisher:
    def __init__(self):

        self.yaw_std_dev = rospy.get_param("~yaw_std_dev")
        self.xyz_std_dev = np.array(rospy.get_param("~xyz_std_dev"))
        self.noisy_tf_parent = rospy.get_param("~noisy_tf_parent")
        self.noisy_tf_child = rospy.get_param("~noisy_tf_child")

        # Transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Odometry publisher
        self.odom_pub = rospy.Publisher("~noisy_odom_out", Odometry, queue_size=10)

        self.last_gt_odom = None
        self.last_noisy_odom = None

        # Odometry subscriber
        self.odom_sub = rospy.Subscriber("~gt_odom_in", Odometry, self.odom_cb)

    def odom_cb(self, odom_msg):

        if self.last_gt_odom is None:
            self.last_gt_odom = odom_msg
            self.last_noisy_odom = odom_msg
            return

        pose_delta = calc_odom_delta(self.last_gt_odom, odom_msg)

        self.last_noisy_odom = self.add_odom_noise(
            self.last_noisy_odom,
            odom_msg,
            pose_delta,
        )

        self.publish_noisy_transform(
            self.noisy_tf_parent,
            self.noisy_tf_child,
            odom_msg.header.stamp,
            self.last_noisy_odom.pose.pose,
        )

        self.odom_pub.publish(self.last_noisy_odom)
        self.last_gt_odom = odom_msg

    def publish_noisy_transform(self, fixed_frame, child_frame, time, noisy_pose):
        """Publish the noisy transform."""
        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = fixed_frame
        t.child_frame_id = child_frame

        # Set position from noisy pose
        t.transform.translation.x = noisy_pose.position.x
        t.transform.translation.y = noisy_pose.position.y
        t.transform.translation.z = noisy_pose.position.z

        # Set orientation from noisy pose
        t.transform.rotation = noisy_pose.orientation

        # Broadcast the noisy transform
        self.br.sendTransform(t)

    def add_odom_noise(
        self,
        prev_noisy_odom,
        cur_gt_odom,
        pose_delta,
    ):

        updated_odom = Odometry()
        updated_odom.header = cur_gt_odom.header

        prev_pose = mat_from_pose(prev_noisy_odom.pose.pose)
        noisy_trans = np.random.normal(scale=self.xyz_std_dev)
        noisy_rot = [0, 0, np.random.normal(scale=self.yaw_std_dev)]
        updated_pose = prev_pose @ pose_delta @ Exp(noisy_trans, noisy_rot)
        updated_odom.pose.pose.position.x = updated_pose[0, 3]
        updated_odom.pose.pose.position.y = updated_pose[1, 3]
        updated_odom.pose.pose.position.z = updated_pose[2, 3]

        q = R.from_matrix(updated_pose[:3, :3]).as_quat()
        updated_odom.pose.pose.orientation.x = q[0]
        updated_odom.pose.pose.orientation.y = q[1]
        updated_odom.pose.pose.orientation.z = q[2]
        updated_odom.pose.pose.orientation.w = q[3]

        q_gt = [
            cur_gt_odom.pose.pose.orientation.x,
            cur_gt_odom.pose.pose.orientation.y,
            cur_gt_odom.pose.pose.orientation.z,
            cur_gt_odom.pose.pose.orientation.w,
        ]

        R_gt = R.from_quat(q_gt).as_matrix()
        R_noisy = updated_pose[:3, :3]

        # TODO: Check this, I don't think we should need to rotate here but maybe a bug in the sim's velocity frame?
        rel_rot = (R_gt.T @ R_noisy).T

        vel = np.array(
            [
                cur_gt_odom.twist.twist.linear.x,
                cur_gt_odom.twist.twist.linear.y,
                cur_gt_odom.twist.twist.linear.z,
            ]
        )

        rot_vel = rel_rot @ vel

        updated_odom.twist.twist.linear.x = rot_vel[0]
        updated_odom.twist.twist.linear.y = rot_vel[1]
        updated_odom.twist.twist.linear.z = rot_vel[2]

        return updated_odom


if __name__ == "__main__":
    rospy.init_node("noisy_tf_publisher_node")
    node = NoisyTransformPublisher()
    rospy.spin()
