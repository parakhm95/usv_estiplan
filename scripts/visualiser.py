#!/usr/bin/env python
from usv_estiplan.msg import OdometryArray
from geometry_msgs.msg import PoseArray
import rospy 

model1_pub = None
model2_pub = None


def model1_predictions_callback(msg):
    global model1_pub
    pub_msg = PoseArray()
    pub_msg.header.stamp = msg.header.stamp
    pub_msg.header.frame_id = msg.header.frame_id

    for i in range(len(msg.odom_array)):
        pub_msg.poses.append(msg.odom_array[i].pose)

    model1_pub.publish(pub_msg)



def model2_predictions_callback(msg):
    global model2_pub
    pub_msg = PoseArray()
    pub_msg.header.stamp = msg.header.stamp
    pub_msg.header.frame_id = msg.header.frame_id

    for i in range(len(msg.odom_array)):
        pub_msg.poses.append(msg.odom_array[i].pose)

    model2_pub.publish(pub_msg)

def visualiser():
    global model1_pub, model2_pub
    rospy.init_node('visualiser', anonymous=True)
    model1_pub=rospy.Publisher('~model1_predictions_vis', PoseArray, queue_size=1)
    model2_pub=rospy.Publisher('~model2_predictions_vis', PoseArray, queue_size=1)
    rospy.Subscriber('~model1_predictions_in', OdometryArray, model1_predictions_callback)
    rospy.Subscriber('~model2_predictions_in', OdometryArray, model2_predictions_callback)
    rospy.spin()


if __name__ == '__main__':
    visualiser()
