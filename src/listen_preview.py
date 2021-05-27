import matplotlib.pyplot as plt
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_matrix, quaternion_matrix
import csv
if __name__ == "__main__":

    rospy.init_node('transform_listening')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    set_inverse = 0
    RINV = None
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                'base_link', 'odom', rospy.Time(0))

            # rotation built in two ways, result identical:
            R = quaternion_matrix(rot)
            print("first R: ")
            print(R)
            # if set_inverse == 0:
            #     RINV = np.linalg.inv(R)
            #     set_inverse = 1
            # print("R: ", R)
            # print("R inverted: ", RINV)
            # R_adjusted = np.matmul(R, RINV)
            # print("Adjusted: ", adjusted)
            # euler = euler_from_matrix(R_adjusted, 'sxyz')

            # euler = tf.transformations.euler_from_quaternion(rot)

        except(tf.LookupException, tf.ConnectivityException):
            print("exception was raised")
            continue

        rate.sleep()
