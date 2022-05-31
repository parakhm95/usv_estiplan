#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math
import random
from mrs_msgs.msg import UavState

model_msg = ModelState()
tag_msg = PoseStamped()
rpy_msg = Vector3()
tag_selected_id = Float64()

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length

# topics for subs and pubs
gazebo_model_name = "wamv"
# bools and variables
tag_publish_rate = 200.0
elap_time = 0.0000
publish_tag_detections = False
random_noise = True
tag_pub = None
tag_selected_id_pub = None
tag_frame = "uav7/fcu"
# freq, amp, phase

wave_pub = rospy.Publisher('/gazebo/set_model_state',
                           ModelState, queue_size=1000)


def UavStateCallback(msg):
    global tag_frame
    tag_frame = msg.header.frame_id


def ProcessTag(msg, i):
    global tag_msg, tag_pub, tag_frame, tag_selected_id_pub, tag_timeout
    tag_msg.header.stamp = msg.header.stamp
    tag_msg.header.frame_id = tag_frame
    tag_msg.pose.position.x = msg.detections[i].pose.pose.pose.position.x
    tag_msg.pose.position.y = msg.detections[i].pose.pose.pose.position.y
    tag_msg.pose.position.z = msg.detections[i].pose.pose.pose.position.z
    tag_msg.pose.orientation.w = msg.detections[i].pose.pose.pose.orientation.w
    tag_msg.pose.orientation.x = msg.detections[i].pose.pose.pose.orientation.x
    tag_msg.pose.orientation.y = msg.detections[i].pose.pose.pose.orientation.y
    tag_msg.pose.orientation.z = msg.detections[i].pose.pose.pose.orientation.z
    transform = tf_buffer.lookup_transform(tag_msg.header.frame_id,
                                           # source frame:
                                           msg.header.frame_id,
                                           # get the tf at the time the pose was valid
                                           msg.header.stamp,
                                           # wait for at most 1 second for transform, otherwise throw
                                           rospy.Duration(0.1))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(
        tag_msg, transform)
    tag_msg.pose = pose_transformed.pose
    tag_pub.publish(tag_msg)
    tag_selected_id_pub.publish(msg.detections[i].id[0])


def callback(msg):
    global tag_pub, tag_msg, tag_selected_id_pub, tag_timeout
    max_size = 0.0
    selected_index = -1
    for i in range(len(msg.detections)):
        if(msg.detections[i].size[0] > max_size):
            max_size = msg.detections[i].size[0]
            selected_index = i
    if selected_index != -1:
        ProcessTag(msg, selected_index)


def bridge():
    global tag_pub, tag_selected_id_pub
    rospy.init_node('april_tag_bridge', anonymous=True)
    rospy.Subscriber("~tag_detections_in", AprilTagDetectionArray, callback)
    rospy.Subscriber("~uav_state_in", UavState, UavStateCallback)
    tag_pub = rospy.Publisher('~base_detections_out',
                              PoseStamped, queue_size=1000)
    tag_selected_id_pub = rospy.Publisher(
        '~selected_tag_id', Float64, queue_size=1000)
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(tag_publish_rate)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        # tf initialization

        bridge()
    except rospy.ROSInterruptException:
        pass
