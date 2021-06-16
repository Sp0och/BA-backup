import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

Data = "Intensity"
Extractor = "KLT"
MASK = "ON"
max_match_distance = "0.3m"
min_distance = "0.1m"
max_cos = "0.2"
smoothing = "1"
length = "100"
num_points = int(length*10)


# calculates average error


def averager(pr_x, pr_y, pr_z, pr_roll, pr_pitch, pr_yaw):
    diffx = pr_x-GT_xs
    diffy = pr_y-GT_ys
    diffz = pr_z-GT_zs
    diffroll = pr_roll-GT_rolls
    diffpitch = pr_pitch-GT_pitchs
    diffyaw = pr_yaw-GT_yaws

    average_x = np.sum(np.abs(diffx))/num_points
    average_y = np.sum(np.abs(diffy))/num_points
    average_z = np.sum(np.abs(diffz))/num_points
    average_roll = np.sum(np.abs(diffroll))/num_points
    average_pitch = np.sum(np.abs(diffpitch))/num_points
    average_yaw = np.sum(np.abs(diffyaw))/num_points

    return average_x, average_y, average_z, average_roll, average_pitch, average_yaw


def get__complete_vector(param1, param2):
    data = pd.read_csv("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/comparison_data/complete_opt_size" +
                       param1+"num_pyramids"+param2+".csv", nrows=1000)
    data_x = pd.DataFrame.to_numpy(data["x"])
    data_y = pd.DataFrame.to_numpy(data["y"])
    data_z = pd.DataFrame.to_numpy(data["z"])
    data_roll = pd.DataFrame.to_numpy(data["roll"])
    data_pitch = pd.DataFrame.to_numpy(data["pitch"])
    data_yaw = pd.DataFrame.to_numpy(data["yaw"])

    return data_x, data_y, data_z, data_roll, data_pitch, data_yaw


def get_step_vector(param1, param2):
    data = pd.read_csv("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/comparison_data/steps_opt_size" +
                       param1+"num_pyramids"+param2+".csv", nrows=1000)
    data_x = pd.DataFrame.to_numpy(data["x"])
    data_y = pd.DataFrame.to_numpy(data["y"])
    data_z = pd.DataFrame.to_numpy(data["z"])
    data_roll = pd.DataFrame.to_numpy(data["roll"])
    data_pitch = pd.DataFrame.to_numpy(data["pitch"])
    data_yaw = pd.DataFrame.to_numpy(data["yaw"])

    return data_x, data_y, data_z, data_roll, data_pitch, data_yaw


if __name__ == "__main__":

    GT_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_steps.csv", nrows=1000
    )

    GT_xs = pd.DataFrame.to_numpy(GT_steps["x"])
    GT_ys = pd.DataFrame.to_numpy(GT_steps["y"])
    GT_zs = pd.DataFrame.to_numpy(GT_steps["z"])
    GT_rolls = pd.DataFrame.to_numpy(GT_steps["roll"])
    GT_pitchs = pd.DataFrame.to_numpy(GT_steps["pitch"])
    GT_yaws = pd.DataFrame.to_numpy(GT_steps["yaw"])

    GT_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_complete.csv", nrows=1000
    )

    GT_xc = pd.DataFrame.to_numpy(GT_overall["x"])
    GT_yc = pd.DataFrame.to_numpy(GT_overall["y"])
    GT_zc = pd.DataFrame.to_numpy(GT_overall["z"])
    GT_rollc = pd.DataFrame.to_numpy(GT_overall["roll"])
    GT_pitchc = pd.DataFrame.to_numpy(GT_overall["pitch"])
    GT_yawc = pd.DataFrame.to_numpy(GT_overall["yaw"])

    odom_file_complete = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_aligned.csv", nrows=1000
    )
    odom_xc = pd.DataFrame.to_numpy(odom_file_complete["x"])
    odom_yc = pd.DataFrame.to_numpy(odom_file_complete["y"])
    odom_zc = pd.DataFrame.to_numpy(odom_file_complete["z"])
    odom_rollc = pd.DataFrame.to_numpy(odom_file_complete["roll"])
    odom_pitchc = pd.DataFrame.to_numpy(odom_file_complete["pitch"])
    odom_yawc = pd.DataFrame.to_numpy(odom_file_complete["yaw"])

    odom_file_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_steps.csv", nrows=1000
    )

    odom_xs = pd.DataFrame.to_numpy(odom_file_steps["x"])
    odom_ys = pd.DataFrame.to_numpy(odom_file_steps["y"])
    odom_zs = pd.DataFrame.to_numpy(odom_file_steps["z"])
    odom_rolls = pd.DataFrame.to_numpy(odom_file_steps["roll"])
    odom_pitchs = pd.DataFrame.to_numpy(odom_file_steps["pitch"])
    odom_yaws = pd.DataFrame.to_numpy(odom_file_steps["yaw"])
