import numpy as np
import pandas as pd
from tf.transformations import euler_from_matrix


def euler_to_rotMat(roll, pitch, yaw):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0, 1]])
    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat


if __name__ == "__main__":
    odom_file_pose = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_aligned.csv", nrows=1000
    )
    odom_xc = pd.DataFrame.to_numpy(odom_file_pose["x"])
    odom_yc = pd.DataFrame.to_numpy(odom_file_pose["y"])
    odom_zc = pd.DataFrame.to_numpy(odom_file_pose["z"])
    odom_rollc = pd.DataFrame.to_numpy(odom_file_pose["roll"])
    odom_pitchc = pd.DataFrame.to_numpy(odom_file_pose["pitch"])
    odom_yawc = pd.DataFrame.to_numpy(odom_file_pose["yaw"])

    odom_file_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_steps.csv", nrows=1000
    )

    odom_xs = pd.DataFrame.to_numpy(odom_file_steps["x"])
    odom_ys = pd.DataFrame.to_numpy(odom_file_steps["y"])
    odom_zs = pd.DataFrame.to_numpy(odom_file_steps["z"])
    odom_rolls = pd.DataFrame.to_numpy(odom_file_steps["roll"])
    odom_pitchs = pd.DataFrame.to_numpy(odom_file_steps["pitch"])
    odom_yaws = pd.DataFrame.to_numpy(odom_file_steps["yaw"])

    R_init = euler_to_rotMat(
        1.6062332390059573, -0.03616810286905262, 2.5137567238980787)
    T_init = np.eye(4)
    T_init[0:3, 0:3] = R_init
    T_init[0:3, 3] = [-102.33882912774837,
                      61.70305342070132, 115.17445561723925]
    R_step = euler_to_rotMat(0.013605646786752863, -
                             0.00522961079305786, -0.002669877365929258)
    T_step = np.eye(4)
    T_step[0:3, 0:3] = R_step
    T_step[0:3, 3] = [0.14325215041225192, -
                      0.019046657825626312, 0.014573579790091884]
    T_after = np.matmul(T_init, T_step)
    R_after = np.copy(T_after)
    R_after[0:3, 3] = 0
    euler_after = euler_from_matrix(R_after, 'sxyz')
    print(T_after[0, 3])
    print(T_after[1, 3])
    print(T_after[2, 3])
    print(euler_after[0])
    print(euler_after[1])
    print(euler_after[2])
