import rospy
import numpy as np
import tf
from tf2_msgs.msg import TFMessage
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_matrix, quaternion_matrix
import csv
import message_filters


def callback_storage(PC):
    print("this got called")
    global first_iteration
    global TINV
    timestamp = PC.header.stamp
    print("timestamp: ", timestamp)
    listener = tf.TransformListener()
    listener.waitForTransform(
        'odom', 'base_link',   timestamp, rospy.Duration(1.0))
    # (trans, rot) = listener.lookupTransform(
    #     'base_link', 'odom',   timestamp)

    if(first_iteration):
        # TINV = storage_1(trans, rot, timestamp, TINV)
        first_iteration = False
    else:
        print("else")
        # TINV = storage_2(trans, rot, timestamp, TINV)


def listener():

    rospy.init_node('subscriber_python', anonymous=True)

    rospy.Subscriber("/points_raw", PointCloud2,
                     callback_storage, queue_size=1)

    # PC_sub = message_filters.Subscriber("/points_raw", PointCloud2)
    # TF_sub = message_filters.Subscriber("/tf", TFMessage)

    # ats = message_filters.ApproximateTimeSynchronizer(
    #     [PC_sub, TF_sub], queue_size=5, slop=0.01, allow_headerless=True)
    # ats.registerCallback(callback_storage)
    # print("this works")

    rospy.spin()


# stores the first iteration and calculates the inverse of that pose
def storage_1(trans, rot, timestamp, TINV):
    # create rotation matrix
    R = quaternion_matrix(rot)

    # their GT pose
    euler_complete = euler_from_matrix(R, 'sxyz')
    writer_complete.writerow(
        [trans[0], trans[1], trans[2], euler_complete[0], euler_complete[1], euler_complete[2], timestamp])
    print("this worked")
    # Complete initial state
    Transform = np.copy(R)
    Transform[0:3, 3] = trans

    # build the first inverse to negate the initial pose
    TINV = np.linalg.inv(Transform)

    # complete with zero start pose:
    # writer_complete.writerow(
    #     [0, 0, 0, 0, 0, 0, timestamp])
    return TINV


# normal storage iteration
def storage_2(trans, rot, timestamp, TINV):
    # create rotation matrix
    R = quaternion_matrix(rot)

    # their GT pose
    euler_complete = euler_from_matrix(R, 'sxyz')
    writer_complete.writerow(
        [trans[0], trans[1], trans[2], euler_complete[0], euler_complete[1], euler_complete[2], timestamp])

    # Complete initial state
    Transform = np.copy(R)
    Transform[0:3, 3] = trans

    # COMPLETE WITH 0 START POSE:
    # T_adjusted_complete = np.matmul(TINV, Transform)
    # trans_complete = np.copy(T_adjusted_complete[0:3, 3])
    # R_adjusted_complete = np.copy(T_adjusted_complete)
    # R_adjusted_complete[0:3, 3] = 0
    # euler_complete = euler_from_matrix(R_adjusted_complete, 'sxyz')

    # writer_complete.writerow(
    #     [trans_complete[0], trans_complete[1], trans_complete[2], euler_complete[0], euler_complete[1], euler_complete[2], timestamp])

    # Iteration STEPS:
    T_adjusted_steps = np.matmul(TINV, Transform)
    trans_steps = np.copy(T_adjusted_steps[0:3, 3])
    R_adjusted_steps = np.copy(T_adjusted_steps)
    R_adjusted_steps[0:3, 3] = 0
    euler_steps = euler_from_matrix(R_adjusted_steps, 'sxyz')

    writer_steps.writerow(
        [trans_steps[0], trans_steps[1], trans_steps[2], euler_steps[0], euler_steps[1], euler_steps[2], timestamp])
    TINV = np.linalg.inv(Transform)
    return TINV


if __name__ == '__main__':

    solution_complete = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/test_complete.csv", "w"
    )

    solution_steps = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/test_steps.csv", "w"
    )

    writer_complete = csv.writer(solution_complete)
    writer_steps = csv.writer(solution_steps)
    writer_steps.writerow(["x", "y", "z", "roll", "pitch", "yaw", "time"])
    writer_complete.writerow(["x", "y", "z", "roll", "pitch", "yaw", "time"])

    first_iteration = True
    TINV = np.identity(4)

    listener()
