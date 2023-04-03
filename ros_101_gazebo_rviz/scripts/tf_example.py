#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion

rospy.init_node("tf_example")
listener = tf.TransformListener()

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    robot_x = trans[0]
    robot_y = trans[1]
    (x, y, z, w) = rot
    (_,_,yaw) = euler_from_quaternion([x, y, z, w])

    print(f"X: {robot_x:.2f}\tY: {robot_y:.2f}\t Yaw: {yaw:.2f}")
    rospy.sleep(0.1)
    