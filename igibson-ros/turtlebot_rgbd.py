#!/usr/bin/python3
import logging
import os

import argparse
import numpy as np
import rospkg
import rospy
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from igibson.envs.igibson_env import iGibsonEnv

from extra_sensors import ExtraVisionSensor

class SimNode:
    def __init__(self):
        rospy.init_node("igibson_sim")
        rospack = rospkg.RosPack()
        path = rospack.get_path("igibson-ros")
        config_filename = os.path.join(path, "turtlebot_rgbd.yaml")
        config_data = yaml.load(open(config_filename, "r"), Loader=yaml.FullLoader)

        self.cmdx = 0.0
        self.cmdy = 0.0

        self.image_pub = rospy.Publisher("/gibson_ros/camera/rgb/image", ImageMsg, queue_size=10)
        self.depth_pub = rospy.Publisher("/gibson_ros/camera/depth/image", ImageMsg, queue_size=10)
        self.lidar_pub = rospy.Publisher("/gibson_ros/lidar/points", PointCloud2, queue_size=10)
        self.depth_raw_pub = rospy.Publisher("/gibson_ros/camera/depth/image_raw", ImageMsg, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.gt_pose_pub = rospy.Publisher("/ground_truth_odom", Odometry, queue_size=10)
        self.camera_info_pub = rospy.Publisher("/gibson_ros/camera/depth/camera_info", CameraInfo, queue_size=10)

        rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.cmd_callback)
        rospy.Subscriber("/reset_pose", PoseStamped, self.tp_robot_callback)

        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        self.env = iGibsonEnv(
            config_file=config_data, mode="headless", action_timestep=1 / 30.0
        )  # assume a 30Hz simulation
        self.env.reset()

        self.tp_time = None

        # Add in additional publishers
        self.left_depth_pub = rospy.Publisher("/gibson_ros/camera/depth/left/image", ImageMsg, queue_size=10)
        self.left_image_pub = rospy.Publisher("/gibson_ros/camera/rgb/left/image", ImageMsg, queue_size=10)
        self.right_depth_pub = rospy.Publisher("/gibson_ros/camera/depth/right/image", ImageMsg, queue_size=10)
        self.right_image_pub = rospy.Publisher("/gibson_ros/camera/rgb/right/image", ImageMsg, queue_size=10)

        # Add in extra sensors. Note that the camera poses are
        # relative to the coordinate frame centred on the default
        # sensor on the robot, aligned with world frame axes.

        left_cam_pos = np.array([0.05, 0.0, 0.0])
        right_cam_pos = np.array([-0.05, 0.0, 0.0])
        left_cam_orn = np.array([0.0, 0.0, 0.383, 0.924])
        right_cam_orn = np.array([0.0, 0.0, -0.383, 0.924])

        self.left_cam_modalities = ("depth",)
        self.right_cam_modalities = ("depth",)
        self.left_cam = ExtraVisionSensor(self.env, self.left_cam_modalities, left_cam_pos, left_cam_orn)
        self.right_cam = ExtraVisionSensor(self.env, self.right_cam_modalities, right_cam_pos, right_cam_orn)


    def run(self, args):
        while not rospy.is_shutdown():
            obs, _, _, _ = self.env.step([self.cmdx, self.cmdy])
            if args.enable_left_right_cams:
                left_obs = self.left_cam.get_obs(self.env)
                right_obs = self.right_cam.get_obs(self.env)


            rgb = (obs["rgb"] * 255).astype(np.uint8)
            normalized_depth = obs["depth"].astype(np.float32)
            depth = normalized_depth * self.env.sensors["vision"].depth_high
            depth_raw_image = (obs["depth"] * 1000).astype(np.uint16)

            image_message = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            depth_message = self.bridge.cv2_to_imgmsg(depth, encoding="passthrough")
            depth_raw_message = self.bridge.cv2_to_imgmsg(depth_raw_image, encoding="passthrough")

            now = rospy.Time.now()

            # Default sensor data
            image_message.header.stamp = now
            depth_message.header.stamp = now
            depth_raw_message.header.stamp = now
            image_message.header.frame_id = "camera_depth_optical_frame"
            depth_message.header.frame_id = "camera_depth_optical_frame"
            depth_raw_message.header.frame_id = "camera_depth_optical_frame"

            self.image_pub.publish(image_message)
            self.depth_pub.publish(depth_message)
            self.depth_raw_pub.publish(depth_raw_message)

            # Extra sensor data
            if args.enable_left_right_cams:
                if "rgb" in self.left_cam_modalities:
                    left_rgb = (left_obs["rgb"] * 255).astype(np.uint8)
                    left_rgb_message = self.bridge.cv2_to_imgmsg(left_depth, encoding="rgb8")
                    self.left_image_pub.publish(left_rgb_message)
                if "rgb" in self.right_cam_modalities:
                    right_rgb = (right_obs["rgb"] * 255).astype(np.uint8)
                    right_rgb_message = self.bridge.cv2_to_imgmsg(right_depth, encoding="rgb8")
                    self.right_image_pub.publish(right_rgb_message)
                if "depth" in self.left_cam_modalities:
                    left_normalized_depth = left_obs["depth"].astype(np.float32)
                    left_depth = left_normalized_depth * self.env.sensors["vision"].depth_high
                    left_depth_message = self.bridge.cv2_to_imgmsg(left_depth, encoding="passthrough")
                    self.left_depth_pub.publish(left_depth_message)
                if "depth" in self.right_cam_modalities:
                    right_normalized_depth = right_obs["depth"].astype(np.float32)
                    right_depth = right_normalized_depth * self.env.sensors["vision"].depth_high
                    right_depth_message = self.bridge.cv2_to_imgmsg(right_depth, encoding="passthrough")
                    self.right_depth_pub.publish(right_depth_message)

            # Default sensor camera info
            msg = CameraInfo(
                height=256,
                width=256,
                distortion_model="plumb_bob",
                D=[0.0, 0.0, 0.0, 0.0, 0.0],
                K=[128, 0.0, 128, 0.0, 128, 128, 0.0, 0.0, 1.0],
                R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                P=[128, 0.0, 128, 0.0, 0.0, 128, 128, 0.0, 0.0, 0.0, 1.0, 0.0],
            )
            msg.header.stamp = now
            msg.header.frame_id = "camera_depth_optical_frame"
            self.camera_info_pub.publish(msg)

            if (self.tp_time is None) or (
                (self.tp_time is not None) and ((rospy.Time.now() - self.tp_time).to_sec() > 1.0)
            ):
                scan = obs["scan"]
                lidar_header = Header()
                lidar_header.stamp = now
                lidar_header.frame_id = "scan_link"

                laser_linear_range = self.env.sensors["scan_occ"].laser_linear_range
                laser_angular_range = self.env.sensors["scan_occ"].laser_angular_range
                min_laser_dist = self.env.sensors["scan_occ"].min_laser_dist
                n_horizontal_rays = self.env.sensors["scan_occ"].n_horizontal_rays

                laser_angular_half_range = laser_angular_range / 2.0
                angle = np.arange(
                    -np.radians(laser_angular_half_range),
                    np.radians(laser_angular_half_range),
                    np.radians(laser_angular_range) / n_horizontal_rays,
                )
                unit_vector_laser = np.array([[np.cos(ang), np.sin(ang), 0.0] for ang in angle])
                lidar_points = unit_vector_laser * (scan * (laser_linear_range - min_laser_dist) + min_laser_dist)

                lidar_message = pc2.create_cloud_xyz32(lidar_header, lidar_points.tolist())
                self.lidar_pub.publish(lidar_message)

            # Odometry
            odom = [
                np.array(self.env.robots[0].get_position()) - np.array(self.env.task.initial_pos),
                np.array(self.env.robots[0].get_rpy()) - np.array(self.env.task.initial_orn),
            ]

            self.br.sendTransform(
                (odom[0][0], odom[0][1], 0),
                tf.transformations.quaternion_from_euler(0, 0, odom[-1][-1]),
                rospy.Time.now(),
                "base_footprint",
                "odom",
            )
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"

            odom_msg.pose.pose.position.x = odom[0][0]
            odom_msg.pose.pose.position.y = odom[0][1]
            (
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w,
            ) = tf.transformations.quaternion_from_euler(0, 0, odom[-1][-1])

            odom_msg.twist.twist.linear.x = (self.cmdx + self.cmdy) * 5
            odom_msg.twist.twist.angular.z = (self.cmdy - self.cmdx) * 5 * 8.695652173913043
            self.odom_pub.publish(odom_msg)

            # Ground truth pose
            gt_pose_msg = Odometry()
            gt_pose_msg.header.stamp = rospy.Time.now()
            gt_pose_msg.header.frame_id = "ground_truth_odom"
            gt_pose_msg.child_frame_id = "base_footprint"

            xyz = self.env.robots[0].get_position()
            rpy = self.env.robots[0].get_rpy()

            gt_pose_msg.pose.pose.position.x = xyz[0]
            gt_pose_msg.pose.pose.position.y = xyz[1]
            gt_pose_msg.pose.pose.position.z = xyz[2]
            (
                gt_pose_msg.pose.pose.orientation.x,
                gt_pose_msg.pose.pose.orientation.y,
                gt_pose_msg.pose.pose.orientation.z,
                gt_pose_msg.pose.pose.orientation.w,
            ) = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

            gt_pose_msg.twist.twist.linear.x = self.cmdx
            gt_pose_msg.twist.twist.angular.z = -self.cmdy

            self.gt_pose_pub.publish(gt_pose_msg)

    def cmd_callback(self, data):
        self.cmdx = data.linear.x
        self.cmdy = -data.angular.z

    def tp_robot_callback(self, data):
        rospy.loginfo("Teleporting robot")
        position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        orientation = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
        self.env.robots[0].reset_new_pose(position, orientation)
        self.tp_time = rospy.Time.now()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable_left_right_cams", default=False, type=bool, 
                        help="Enable additional left/right cameras on robot")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    node = SimNode(args)
    node.run(args)
