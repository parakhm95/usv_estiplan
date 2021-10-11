#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
import numpy as np
import tf
import math

model_msg = ModelState()
tag_msg = PoseStamped()

publish_tag_detections = False
tag_topic = "/uav1/rs_d435/color/base_detections"
gazebo_model_name = "wamv"
tag_publish_rate = 25.0
wave_number = 10
CONSTANT_OFFSET_X = 0.0
CONSTANT_OFFSET_Y = 0.0
CONSTANT_OFFSET_Z = 6.0
elap_time =  0.0000
# freq, amp, phase
z_wave = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
roll_wave = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
pitch_wave =np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
yaw_wave = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])

wave_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1000)
tag_pub = rospy.Publisher(tag_topic, PoseStamped, queue_size=1000)



def callback(msg):
    pose_msg.x = msg.pose.position.x
    pose_msg.y = msg.pose.position.y
    pose_msg.z = msg.pose.position.z
    pose_msg.heading = 0.0
    wave_pub.publish(pose_msg)

def bridge():
    rospy.init_node('gazebo_wave_generator', anonymous=True)
    # rospy.Subscriber("/uav1/rs_d435/color/base_detections", PoseStamped, callback)
    rate = rospy.Rate(tag_publish_rate)
    while not rospy.is_shutdown():
        model_msg.pose.position.x = CONSTANT_OFFSET_X
        model_msg.pose.position.y = CONSTANT_OFFSET_Y
        model_msg.pose.position.z = CONSTANT_OFFSET_Z

        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        model_msg.pose.position.z = 0.0
        elap_time =  rospy.Time.now().to_sec()
        for i in range(wave_number):
            model_msg.pose.position.z += z_wave[i,1] * math.sin((2* math.pi *z_wave[i,0]*elap_time)+z_wave[i,2])
            roll += roll_wave[i,1] * math.sin((2* math.pi *roll_wave[i,0]*elap_time)+roll_wave[i,2])
            pitch += pitch_wave[i,1] * math.sin((2* math.pi *pitch_wave[i,0]*elap_time)+pitch_wave[i,2])
            yaw += yaw_wave[i,1] * math.sin((2* math.pi *yaw_wave[i,0]*elap_time)+yaw_wave[i,2])

            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#type(pose) = geometry_msgs.msg.Pose
        model_msg.pose.orientation.x = quaternion[0]
        model_msg.pose.orientation.y = quaternion[1]
        model_msg.pose.orientation.z = quaternion[2]
        model_msg.pose.orientation.w = quaternion[3]

        wave_pub.publish(model_msg)
        if(publish_tag_detections):
            tag_msg.pose.position.x = model_msg.pose.position.x
            tag_msg.pose.position.y = model_msg.pose.position.y
            tag_msg.pose.position.z = model_msg.pose.position.z

            tag_msg.pose.orientation.w = model_msg.pose.orientation.w
            tag_msg.pose.orientation.x = model.msg.pose.orientation.x
            tag_msg.pose.orientation.y = model.msg.pose.orientation.y
            tag_msg.pose.orientation.z = model.msg.pose.orientation.Z
            
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        bridge()
    except rospy.ROSInterruptException:
        pass
