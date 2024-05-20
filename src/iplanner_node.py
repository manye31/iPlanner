#!/usr/bin/env python
# ======================================================================
# Copyright (c) 2023 Fan Yang
# Robotic Systems Lab, ETH Zurich
# All rights reserved.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
# ======================================================================

import os
import PIL
import sys
import time
import torch
import numpy as np
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
import ament_index_python

from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped

from iplanner.ip_algo import IPlannerAlgo
from iplanner.rosutil import ROSArgparse
from ament_index_python.resources import get_resource

package_name = 'iplanner'
resource_name = 'package'
# pack_path = get_resource(resource_name, package_name).split(os.pathsep)[0]
pack_path = ament_index_python.get_package_share_directory('iplanner')
planner_path = os.path.join(pack_path,'iplanner')
print(pack_path)
print(planner_path)
sys.path.append(pack_path)
sys.path.append(planner_path)
print(pack_path)
print(planner_path)


class iPlannerNode(Node):
    # def __init__(self, options):
    def __init__(self, args):
        super().__init__("iplanner_node")
        self.config(args)

        # init planner algo class
        self.iplanner_algo = IPlannerAlgo(args=args)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        time.sleep(2.5) # wait for tf listener to be ready

        self.image_time = self.get_clock().now()
        self.is_goal_init = False
        self.ready_for_planning = False

        # planner status
        self.planner_status = Int16()
        self.planner_status.data = 0
        self.is_goal_processed = False
        self.is_smartjoy = False

        # fear reaction
        self.fear_buffter = 0
        self.is_fear_reaction = False

        # process time
        self.timer_data = Float32()
        
        self.camera_sub = self.create_subscription(Image, self.image_topic, self.imageCallback, 10)
        self.goal_sub = self.create_subscription(PointStamped, self.goal_topic, self.goalCallback, 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joyCallback, 10)

        timer_topic = '/ip_timer'
        status_topic = '/ip_planner_status'
        
        # planning status topics
        self.timer_pub = self.create_publisher(Float32, timer_topic, 10)
        self.status_pub = self.create_publisher(Int16, status_topic, 10)
        self.path_pub  = self.create_publisher(Path, self.path_topic, 10)
        self.fear_path_pub = self.create_publisher(Path, self.path_topic + "_fear", 10)
        self.spin_pub = self.create_publisher(Float32, "spin", 10)

        self.timer = self.create_timer(1.0/self.main_freq, self.plan)

        self.get_logger().info('iPlanner ready')
        

    def config(self, args):
        self.main_freq   = args.main_freq
        self.model_save  = args.model_save
        self.image_topic = args.depth_topic
        self.goal_topic  = args.goal_topic
        self.path_topic  = args.path_topic
        self.frame_id    = args.robot_id
        self.world_id    = args.world_id
        self.uint_type   = args.uint_type
        self.image_flip  = args.image_flip
        self.conv_dist   = args.conv_dist
        self.depth_max   = args.depth_max
        # fear reaction
        self.is_fear_act = args.is_fear_act
        self.buffer_size = args.buffer_size
        self.ang_thred   = args.angular_thred
        self.track_dist  = args.track_dist
        self.joyGoal_scale = args.joyGoal_scale
        return 

    def plan(self):
        # self.get_logger().info("ok? {} and {}".format(self.ready_for_planning,self.is_goal_init))
        if rclpy.ok():
            msg = Float32()
            msg.data = 2001.
            self.spin_pub.publish(msg)
            self.get_logger().info("{} and {}".format(self.ready_for_planning,self.is_goal_init))
            if self.ready_for_planning and self.is_goal_init:
                # main planning starts
                cur_image = self.img.copy()
                start = time.time()
                # Network Planning
                self.get_logger().info("goal_rb {}".format(self.goal_rb))
                # print(self.goal_robot_frame.header.stamp)
                # print(self.msg.header.stamp)
                # print(self.get_clock().now())
                # self.preds, self.waypoints, fear_output, _ = self.iplanner_algo.plan(cur_image, self.goal_rb)
                self.waypoints = torch.tensor([[0., 0., 0.],[0.03, 0.03, 0.0], [0.06, 0.06, 0.0]])
                fear_output = torch.tensor([[0.0]])
                end = time.time()
                self.timer_data.data = (end - start) * 1000
                self.timer_pub.publish(self.timer_data)
                # check goal less than converage range
                if (np.sqrt(self.goal_rb[0][0]**2 + self.goal_rb[0][1]**2) < self.conv_dist) and self.is_goal_processed and (not self.is_smartjoy):
                    self.ready_for_planning = False
                    self.is_goal_init = False
                    # planner status -> Success
                    if self.planner_status.data == 0:
                        self.planner_status.data = 1
                        self.status_pub.publish(self.planner_status)

                    self.get_logger().info("Goal Arrived")
                self.fear = torch.tensor([[0.0]], device=fear_output.device)
                if self.is_fear_act:
                    self.fear = fear_output
                    is_track_ahead = self.isForwardTraking(self.waypoints)
                    self.fearPathDetection(self.fear, is_track_ahead)
                    if self.is_fear_reaction:
                        self.get_logger().warning(2.0, "current path prediction is invaild.")
                        # planner status -> Fails
                        if self.planner_status.data == 0:
                            self.planner_status.data = -1
                            self.status_pub.publish(self.planner_status)
                self.pubPath(self.waypoints, self.is_goal_init)
                # self.is_goal_init = False

    def pubPath(self, waypoints, is_goal_init=True):
        path = Path()
        fear_path = Path()
        # print(waypoints.squeeze(0))
        if is_goal_init:
            for p in waypoints.squeeze(0):
                pose = PoseStamped()
                pose.pose.position.x = float(p[0])
                pose.pose.position.y = float(p[1])
                pose.pose.position.z = float(p[2])
                path.poses.append(pose)
        # add header
        path.header.frame_id = fear_path.header.frame_id = self.frame_id
        path.header.stamp = fear_path.header.stamp = self.image_time
        # publish fear path
        if self.is_fear_reaction:
            fear_path.poses = path.poses.copy()
            path.poses = path.poses[:1]
        # publish path
        self.fear_path_pub.publish(fear_path)
        self.path_pub.publish(path)
        return

    def fearPathDetection(self, fear, is_forward):
        if fear > 0.5 and is_forward:
            if not self.is_fear_reaction:
                self.fear_buffter = self.fear_buffter + 1
        elif self.is_fear_reaction:
            self.fear_buffter = self.fear_buffter - 1
        if self.fear_buffter > self.buffer_size:
            self.is_fear_reaction = True
        elif self.fear_buffter <= 0:
            self.is_fear_reaction = False
        return None

    def isForwardTraking(self, waypoints):
        xhead = np.array([1.0, 0])
        phead = None
        for p in waypoints.squeeze(0):
            if torch.norm(p[0:2]).item() > self.track_dist:
                phead = np.array([p[0].item(), p[1].item()])
                phead /= np.linalg.norm(phead)
                break
        if phead is None or phead.dot(xhead) > 1.0 - self.ang_thred:
            return True
        return False

    def joyCallback(self, joy_msg):
        if joy_msg.buttons[4] > 0.9:
            self.getlogger().info("Switch to Smart Joystick mode ...")
            self.is_smartjoy = True
            # reset fear reaction
            self.fear_buffter = 0
            self.is_fear_reaction = False
        if self.is_smartjoy:
            if np.sqrt(joy_msg.axes[3]**2 + joy_msg.axes[4]**2) < 1e-3:
                # reset fear reaction
                self.fear_buffter = 0
                self.is_fear_reaction = False
                self.ready_for_planning = False
                self.is_goal_init = False
            else:
                joy_goal = PointStamped()
                joy_goal.header.frame_id = self.frame_id
                joy_goal.point.x = joy_msg.axes[4] * self.joyGoal_scale
                joy_goal.point.y = joy_msg.axes[3] * self.joyGoal_scale
                joy_goal.point.z = 0.0
                joy_goal.header.stamp = self.Time.now()
                self.goal_pose = joy_goal
                self.is_goal_init = True
                self.is_goal_processed = False
        return

    def goalCallback(self, msg):
        # self.get_logger().info("Received a new goal %s: %d"%(msg.header.frame_id, msg.header.stamp.nanosec+msg.header.stamp.sec*1e9))
        self.get_logger().info("Recevied a new goal")
        self.goal_pose = msg
        self.is_smartjoy = False
        self.is_goal_init = True
        self.is_goal_processed = False
        # reset fear reaction
        self.fear_buffter = 0
        self.is_fear_reaction = False
        # reste planner status
        self.planner_status.data = 0
        return

    def imageCallback(self, msg):
        # self.get_logger().info("Received image %s: %d"%(msg.header.frame_id, msg.header.stamp.nanosec+msg.header.stamp.sec*1e9))
        self.image_time = msg.header.stamp
        frame = np.asarray(msg.data)
        frame[~np.isfinite(frame)] = 0
        if self.uint_type:
            frame = frame / 1000.0
        frame[frame > self.depth_max] = 0.0
        # DEBUG - Visual Image
        # img = PIL.Image.fromarray((frame * 255 / np.max(frame[frame>0])).astype('uint8'))
        # img.show()
        if self.image_flip:
            frame = PIL.Image.fromarray(frame)
            self.img = np.array(frame.transpose(PIL.Image.ROTATE_180))
        else:
            self.img = frame

        self.get_logger().info(f"[imageCallback] Goal initialized: {self.is_goal_init}")
        if self.is_goal_init:
            # self.get_logger().info("\n\ndoing the goal transform")
            goal_robot_frame = self.goal_pose
            # print(f"\n\ngoal_pose {goal_robot_frame}")
            if not self.goal_pose.header.frame_id == self.frame_id:
                try:
                    # goal_robot_frame.header.stamp = self.tf_listener.getLatestCommonTime(self.goal_pose.header.frame_id,
                    #                                                                      self.frame_id)
                    # goal_robot_frame = self.tf_listener.transformPoint(self.frame_id, goal_robot_frame)
                    # import pdb; pdb.set_trace()
                    transform_time = self.tf_buffer.get_latest_common_time(self.goal_pose.header.frame_id, self.frame_id)
                    goal_robot_frame.header.stamp = transform_time.to_msg()
                    # print("\n\n{}".format(transform_time))
                    # print("\n\n{}\n\n".format(transform_time.nanoseconds))
                    goal_robot_frame = self.tf_buffer.transform(goal_robot_frame, self.frame_id)
                    # goal_transform = self.tf_buffer.lookup_transform(self.frame_id, self.goal_pose.header.frame_id, rclpy.time.Time())
                    # print(f"\n\ngoal_transform {goal_transform}")
                    # goal_robot_frame = tf2_geometry_msgs.do_transform_point(self.goal_pose, goal_transform)
                    # print(f"\n\ngoal_robot_frame {goal_robot_frame}")
                    self.goal_robot_frame = goal_robot_frame

                except TransformException as ex:
                    self.get_logger().error("Fail to transfer the goal ({}) into base frame ({}): {}".format(self.goal_pose.header.frame_id,self.frame_id,ex))
                    return
            goal_robot_frame = torch.tensor([goal_robot_frame.point.x, goal_robot_frame.point.y, goal_robot_frame.point.z], dtype=torch.float32)[None, ...]
            # goal_robot_frame = torch.tensor([goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.translation.z], dtype=torch.float32)[None, ...]
            self.goal_rb = goal_robot_frame
            self.msg = Path()
            self.msg.header.stamp = self.get_clock().now().to_msg()
        else:
            return
        self.ready_for_planning = True
        self.is_goal_processed  = True
        return

def main(args=None):
    rclpy.init(args=args)
    # options = rclpy.node.NodeOptions(
    #     namespace='iplanner',
    #     arguments='iplanner_node'
    # )

    node_name = "iplanner_node"
    node = rclpy.create_node(node_name)
    # node = iPlannerNode(options)

    parser = ROSArgparse(node=node)
    parser.add_argument('main_freq',         type=int,   default=1,                          help="Main frequency of the path planner.")
    parser.add_argument('model_save',        type=str,   default='/home/micahnye/mdr-ws/src/planning/iplanner/iplanner/models/plannernet.pt',    help="Path to the saved model.")
    parser.add_argument('crop_size',         type=tuple, default=[360,640],                  help='Size to crop the image to.')
    parser.add_argument('uint_type',         type=bool,  default=False,                      help="Determines if the image is in uint type.")
    parser.add_argument('depth_topic',       type=str,   default='/rgbd_camera/depth/image', help='Topic for depth image.')
    parser.add_argument('goal_topic',        type=str,   default='/way_point',               help='Topic for goal waypoints.')
    parser.add_argument('path_topic',        type=str,   default='/path',                    help='Topic for iPlanner path.')
    parser.add_argument('robot_id',          type=str,   default='vehicle',                     help='TF frame ID for the robot.')
    parser.add_argument('world_id',          type=str,   default='map',                     help='TF frame ID for the world.')
    parser.add_argument('depth_max',         type=float, default=10.0,                       help='Maximum depth distance in the image.')
    parser.add_argument('image_flip',        type=bool,  default=True,                       help='Indicates if the image is flipped.')
    parser.add_argument('conv_dist',         type=float, default=0.25,                        help='Convergence range to the goal.')
    parser.add_argument('is_fear_act',       type=bool,  default=True,                       help='Indicates if fear action is enabled.')
    parser.add_argument('buffer_size',       type=int,   default=10,                         help='Buffer size for fear reaction.')
    parser.add_argument('angular_thred',     type=float, default=0.3,                        help='Angular threshold for turning.')
    parser.add_argument('track_dist',        type=float, default=0.5,                        help='Look-ahead distance for path tracking.')
    parser.add_argument('joyGoal_scale',     type=float, default=0.5,                        help='Scale for joystick goal distance.')
    parser.add_argument('sensor_offset_x',   type=float, default=0.0,                        help='Sensor offset on the X-axis.')
    parser.add_argument('sensor_offset_y',   type=float, default=0.0,                        help='Sensor offset on the Y-axis.')

    args = parser.parse_args()

    node = iPlannerNode(args)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
