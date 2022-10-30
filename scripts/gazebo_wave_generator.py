#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3
import numpy as np
import tf
import math
import random

model_msg = ModelState()
tag_msg = PoseStamped()
rpy_msg = Vector3()

publish_tag_detections = False
random_noise = True
tag_topic = "/uav1/rs_d435/color/base_detections"
gazebo_model_name = "wamv"
tag_publish_rate = 30.0
wave_number = 10
CONSTANT_OFFSET_X = 3.0
CONSTANT_OFFSET_Y = 3.0
CONSTANT_OFFSET_Z = 2.0
x_velocity = 0.0    
y_velocity = 0.0    
elap_time = 0.0000
# freq, amp, phase
z_wave = np.array([[0.11, 0.28, 0.0], [0.05, 0.28, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
roll_wave = np.array([[0.1, 0.18, 0.0], [0.12, 0.02, 0.0], [0.14, 0.06, 0.0], [0.08, 0.2, 0.0], [
                     0.11, 0.28, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
pitch_wave = np.array([[0.1, 0.18, 0.0], [0.08, 0.03, 0.0], [0.02, 0.08, 0.0], [0.05, 0.2, 0.0], [
                      0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
yaw_wave = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
                    0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

wave_pub = rospy.Publisher('/gazebo/set_model_state',
                           ModelState, queue_size=1000)
tag_pub = rospy.Publisher(tag_topic, PoseStamped, queue_size=1000)
tag_rpy_pub = rospy.Publisher(tag_topic+'_rpy', Vector3, queue_size=1000)


def callback(msg):
    pose_msg.x = msg.pose.position.x
    pose_msg.y = msg.pose.position.y
    pose_msg.z = msg.pose.position.z
    pose_msg.heading = 0.0
    wave_pub.publish(pose_msg)


def bridge():
    global x_velocity, y_velocity
    rospy.init_node('gazebo_wave_generator', anonymous=True)
    # rospy.Subscriber("/uav1/rs_d435/color/base_detections", PoseStamped, callback)
    rate = rospy.Rate(tag_publish_rate)
    while not rospy.is_shutdown():
        model_msg.reference_frame = 'world'
        model_msg.pose.position.z = CONSTANT_OFFSET_Z
        acc_max = 3.5
        random_x_acc = -acc_max + acc_max*random.random()
        random_y_acc = -acc_max + acc_max*random.random()
        dt = 0.033
        x_velocity += random_x_acc*dt
        y_velocity += random_y_acc*dt
        model_msg.pose.position.x = CONSTANT_OFFSET_X + x_velocity*dt
        model_msg.pose.position.y = CONSTANT_OFFSET_Y + y_velocity*dt

        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        # model_msg.pose.position.z = 0.0
        elap_time = rospy.Time.now().to_sec()
        for i in range(wave_number):
            model_msg.pose.position.z += z_wave[i,1] * math.sin((2* math.pi *z_wave[i,0]*elap_time)+z_wave[i,2])
            roll += roll_wave[i, 1] * \
                math.sin(
                    (2 * math.pi * roll_wave[i, 0]*elap_time)+roll_wave[i, 2])
            pitch += pitch_wave[i, 1] * math.sin(
                (2 * math.pi * pitch_wave[i, 0]*elap_time)+pitch_wave[i, 2])
            yaw += yaw_wave[i, 1] * \
                math.sin(
                    (2 * math.pi * yaw_wave[i, 0]*elap_time)+yaw_wave[i, 2])

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        model_msg.pose.orientation.x = quaternion[0]
        model_msg.pose.orientation.y = quaternion[1]
        model_msg.pose.orientation.z = quaternion[2]
        model_msg.pose.orientation.w = quaternion[3]
        model_msg.model_name = gazebo_model_name
        wave_pub.publish(model_msg)

        if(publish_tag_detections):
            tag_msg.pose.position.x = model_msg.pose.position.x
            tag_msg.pose.position.y = model_msg.pose.position.y
            # 1.5 is the height of the landing platform from boat origin
            tag_msg.pose.position.z = model_msg.pose.position.z + 1.30
            rpy_msg.x = roll
            rpy_msg.y = pitch
            rpy_msg.z = yaw

            if(random_noise):
                roll += -0.02 + 0.04*random.random()
                pitch += -0.02 + 0.04*random.random()
                yaw += -0.02 + 0.04*random.random()
                quaternion = tf.transformations.quaternion_from_euler(
                    roll, pitch, yaw)
                tag_msg.pose.orientation.x = quaternion[0]
                tag_msg.pose.orientation.y = quaternion[1]
                tag_msg.pose.orientation.z = quaternion[2]
                tag_msg.pose.orientation.w = quaternion[3]

                rpy_msg.x = roll
                rpy_msg.y = pitch
                rpy_msg.z = yaw

                model_msg.pose.position.z = tag_msg.pose.position.z
                model_msg.model_name = "sphere"

            tag_pub.publish(tag_msg)
            tag_rpy_pub.publish(rpy_msg)

        # print("I am running")

        # rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    try:
        bridge()
    except rospy.ROSInterruptException:
        pass
