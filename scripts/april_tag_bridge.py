#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math
import random

model_msg = ModelState()
tag_msg = PoseStamped()
rpy_msg = Vector3()

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length

# topics for subs and pubs
tag_topic = "/uav1/rs_d435/color/base_detections"
gazebo_model_name = "wamv"
# bools and variables
tag_publish_rate = 200.0
elap_time =  0.0000
publish_tag_detections = False
random_noise = True
# freq, amp, phase

wave_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1000)
tag_pub = rospy.Publisher(tag_topic, PoseStamped, queue_size=1000)


def callback(msg):
    for i in range(len(msg.detections)):
        if(msg.detections[i].id[0] == 2):
            tag_msg.header.frame_id = 'uav1/gps_origin'
            tag_msg.pose.position.x = msg.detections[i].pose.pose.pose.position.x
            tag_msg.pose.position.y = msg.detections[i].pose.pose.pose.position.y
            tag_msg.pose.position.z = msg.detections[i].pose.pose.pose.position.z
            tag_msg.pose.orientation.w = msg.detections[i].pose.pose.pose.orientation.w
            tag_msg.pose.orientation.x = msg.detections[i].pose.pose.pose.orientation.x
            tag_msg.pose.orientation.y = msg.detections[i].pose.pose.pose.orientation.y
            tag_msg.pose.orientation.z = msg.detections[i].pose.pose.pose.orientation.z
            transform = tf_buffer.lookup_transform('uav1/gps_origin',
                                            # source frame:
                                            msg.header.frame_id,
                                            # get the tf at the time the pose was valid
                                            msg.header.stamp,
                                            # wait for at most 1 second for transform, otherwise throw
                                            rospy.Duration(0.1))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(tag_msg, transform)
            tag_msg.pose = pose_transformed.pose
            tag_pub.publish(tag_msg)

def bridge():
    rospy.init_node('april_tag_bridge', anonymous=True)
    rospy.Subscriber("/uav1/rs_d435/color/tag_detections", AprilTagDetectionArray, callback)
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
