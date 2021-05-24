import matplotlib.pyplot as plt
import rospy
import tf
import numpy as np
import csv
if __name__ == "__main__":

    s_t_x = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_x.csv", 'w')
    s_t_y = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_y.csv", 'w')
    s_t_z = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_z.csv", 'w')
    s_r_x = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_yaw.csv", 'w')
    s_r_y = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_pitch.csv", 'w')
    s_r_z = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_roll.csv", 'w')

    writer_tx = csv.writer(s_t_x)
    writer_ty = csv.writer(s_t_y)
    writer_tz = csv.writer(s_t_z)
    writer_rx = csv.writer(s_r_x)
    writer_ry = csv.writer(s_r_y)
    writer_rz = csv.writer(s_r_z)
    rospy.init_node('transform_listening')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    count = 0
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                'base_link', 'odom', rospy.Time(0))
            print("translation ", count, ": ", trans)
            print("rotation: ", count, ": ", rot)
            writer_tx.writerow([trans[0], ""])
            writer_ty.writerow([trans[1], ""])
            writer_tz.writerow([trans[2], ""])
            euler = tf.transformations.euler_from_quaternion(rot)
            writer_rx.writerow([euler[2], ""])
            writer_ry.writerow([euler[1], ""])
            writer_rz.writerow([euler[0], ""])
            print("Euler: ", euler)
            count = count + 1
        except(tf.LookupException, tf.ConnectivityException):
            print("exception was raised")
            continue

        rate.sleep()

    s_t_x.close()
    s_t_y.close()
    s_t_z.close()
    s_r_x.close()
    s_r_y.close()
    s_r_z.close()
