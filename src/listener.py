import matplotlib.pyplot as plt
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_matrix, quaternion_matrix
import csv
if __name__ == "__main__":

    # for plotting
    s_t_x = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_x.csv", 'w')
    s_t_y = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_y.csv", 'w')
    s_t_z = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_z.csv", 'w')
    s_r_roll = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_yaw.csv", 'w')
    s_r_pitch = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_pitch.csv", 'w')
    s_r_yaw = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_roll.csv", 'w')
    writer_tx = csv.writer(s_t_x)
    writer_ty = csv.writer(s_t_y)
    writer_tz = csv.writer(s_t_z)
    writer_s_roll = csv.writer(s_r_roll)
    writer_s_pitch = csv.writer(s_r_pitch)
    writer_s_yaw = csv.writer(s_r_yaw)

    # all in one for publishing paper solution
    # complete_sol = open(
    #     "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/complete_solution.csv", 'w')
    # writer_complete = csv.writer(complete_sol)
    # writer_complete.writerow(
    #     ["x", "y", "z", "qx", "qy", "qz", "qw"])

    rospy.init_node('transform_listening')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    set_inverse = 1
    RINV = None

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                'odom', 'base_link', rospy.Time(0))
            writer_tx.writerow([trans[0]])
            writer_ty.writerow([trans[1]])
            writer_tz.writerow([trans[2]])
            R = quaternion_matrix(rot)
            # if set_inverse == 1:
            #     RINV = np.linalg.inv(R)
            #     set_inverse = 0
            # R_adjusted = np.matmul(RINV, R)
            # print("R: ", R)
            # print("R inverted: ", RINV)
            # print("Adjusted: ", R_adjusted)
            euler = euler_from_matrix(R, 'sxyz')

            writer_s_roll.writerow([euler[2]])
            writer_s_pitch.writerow([euler[1]])
            writer_s_yaw.writerow([euler[0]])

            # writer_complete.writerow([trans[0], trans[1], trans[2],
            #                           rot[0], rot[1], rot[2], rot[3]])

            print("storing data...")
        except(tf.LookupException, tf.ConnectivityException):
            print("exception was raised")
            continue

        rate.sleep()

    s_t_x.close()
    s_t_y.close()
    s_t_z.close()
    s_r_roll.close()
    s_r_pitch.close()
    s_r_yaw.close()

    # complete_sol.close()
