import rospy
import tf
import numpy as np
from tf.transformations import euler_from_matrix, quaternion_matrix
import csv
if __name__ == "__main__":

    solution_complete = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_complete.csv", "w"
    )
    solution_steps = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_steps.csv", "w"
    )
    # with timestamps
    # solution_complete = open(
    #     "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_complete_" +
    #     time.strftime("%d%m%Y-%H%M")+".csv", "w"
    # )
    # solution_steps = open(
    #     "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_steps_" +
    #     time.strftime("%d%m%Y-%H%M")+".csv", "w"
    # )

    writer_complete = csv.writer(solution_complete)
    writer_steps = csv.writer(solution_steps)

    # tf data in quaternions for python to odom:
    # complete_sol = open(
    #     "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_odometry_data.csv", 'w')
    # writer_complete = csv.writer(complete_sol)
    # writer_complete.writerow(
    #     ["x", "y", "z", "qx", "qy", "qz", "qw"])
    writer_steps.writerow(["x", "y", "z", "roll", "pitch", "yaw", "time"])
    writer_complete.writerow(["x", "y", "z", "roll", "pitch", "yaw", "time"])

    rospy.init_node('transform_listening')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    first_iteration = True
    RINV_complete = None
    RINV_steps = None
    TINV_complete = None
    TINV_steps = None

    while not rospy.is_shutdown():
        try:
            # get position and quaternions from TF
            (trans, rot) = listener.lookupTransform(
                'odom', 'base_link', rospy.Time(0))

            # create rotation matrix
            R = quaternion_matrix(rot)

            # their GT pose
            euler_complete = euler_from_matrix(R, 'sxyz')
            writer_complete.writerow(
                [trans[0], trans[1], trans[2], euler_complete[0], euler_complete[1], euler_complete[2], rospy.Time.now()])

            # Complete initial state
            Transform = np.copy(R)
            Transform[0:3, 3] = trans

            if first_iteration:
                # build the first inverse to negate the initial pose
                TINV_complete = np.linalg.inv(Transform)
                TINV_steps = np.linalg.inv(Transform)
                # complete with zero start pose:
                # writer_complete.writerow(
                #     [0, 0, 0, 0, 0, 0, rospy.Time.now()])
                first_iteration = False

            else:

                # complete with 0 start pose:
                # T_adjusted_complete = np.matmul(TINV_complete, Transform)
                # trans_complete = np.copy(T_adjusted_complete[0:3, 3])
                # R_adjusted_complete = np.copy(T_adjusted_complete)
                # R_adjusted_complete[0:3, 3] = 0
                # euler_complete = euler_from_matrix(R_adjusted_complete, 'sxyz')

                # writer_complete.writerow(
                #     [trans_complete[0], trans_complete[1], trans_complete[2], euler_complete[0], euler_complete[1], euler_complete[2], rospy.Time.now()])

                # Iteration STEPS:
                T_adjusted_steps = np.matmul(TINV_steps, Transform)
                trans_steps = np.copy(T_adjusted_steps[0:3, 3])
                R_adjusted_steps = np.copy(T_adjusted_steps)
                R_adjusted_steps[0:3, 3] = 0
                euler_steps = euler_from_matrix(R_adjusted_steps, 'sxyz')

                writer_steps.writerow(
                    [trans_steps[0], trans_steps[1], trans_steps[2], euler_steps[0], euler_steps[1], euler_steps[2], rospy.Time.now()])
                TINV_steps = np.linalg.inv(Transform)

            print("storing data...")
        except(tf.LookupException, tf.ConnectivityException):
            print("exception was raised")
            continue

        rate.sleep()

    solution_complete.close()
    solution_steps.close()
    # complete_sol.close()
