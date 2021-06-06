from math import degrees
import matplotlib.pyplot as plt
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_matrix, quaternion_matrix
import csv
from scipy.spatial.transform import Rotation
if __name__ == "__main__":

    rospy.init_node('transform_listening')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    set_inverse = 0
    RINV = None
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                'odom', 'base_link', rospy.Time(0))

            # rotation built in two ways, result identical:
            if set_inverse == 0:
                R = quaternion_matrix(rot)
                euler = euler_from_matrix(R, 'sxyz')
                print("trans: ", trans)
                T = np.copy(R)
                T[0:3, 3] = trans
                print("T: ", T)
                TINV = np.linalg.inv(T)
                print("T inverse: ", TINV)
                # Euler = np.array([euler[0], euler[1], euler[2]])
                # print("Euler: ", euler[0], euler[1], euler[2])
                # Euler = Euler * 57.2858
                # Euler = Euler / 2
                # r = Rotation.from_euler("XYZ", Euler, degrees=True)
                # print("r: ", r.as_matrix())
                set_inverse = 1
            # print("Timestamp now: ", rospy.Time.now())
            # R_adjusted = np.matmul(R, RINV)
            # print("Adjusted: ", adjusted)

            # euler = tf.transformations.euler_from_quaternion(rot)

        except(tf.LookupException, tf.ConnectivityException):
            print("exception was raised")
            continue

        rate.sleep()
