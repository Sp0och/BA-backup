import numpy as np
import pandas as pd

Data = "Intensity"
Extractor = "ORB"
MASK = "ON"
max_match_distance = "0.3m"
min_distance = "0.1m"
max_cos = "0.2"
smoothing = "1"
length = "150"

changing_parameter = "best"
num_points = int(length)*10


# calculates average error


def averager_step(pr_x, pr_y, pr_z, pr_roll, pr_pitch, pr_yaw):
    step_diffx = pr_x-GT_xs
    step_diffy = pr_y-GT_ys
    step_diffz = pr_z-GT_zs
    step_diffroll = pr_roll-GT_rolls
    step_diffpitch = pr_pitch-GT_pitchs
    step_diffyaw = pr_yaw-GT_yaws

    average_x = np.sum(np.abs(step_diffx))/num_points
    average_y = np.sum(np.abs(step_diffy))/num_points
    average_z = np.sum(np.abs(step_diffz))/num_points
    average_roll = np.sum(np.abs(step_diffroll))/num_points
    average_pitch = np.sum(np.abs(step_diffpitch))/num_points
    average_yaw = np.sum(np.abs(step_diffyaw))/num_points

    return average_x, average_y, average_z, average_roll, average_pitch, average_yaw


def averager_pose(pr_x, pr_y, pr_z, pr_roll, pr_pitch, pr_yaw):
    pose_diffx = pr_x-GT_xs
    pose_diffy = pr_y-GT_ys
    pose_diffz = pr_z-GT_zs
    pose_diffroll = pr_roll-GT_rolls
    pose_diffpitch = pr_pitch-GT_pitchs
    pose_diffyaw = pr_yaw-GT_yaws

    average_x = np.sum(np.abs(pose_diffx))/num_points
    average_y = np.sum(np.abs(pose_diffy))/num_points
    average_z = np.sum(np.abs(pose_diffz))/num_points
    average_roll = np.sum(np.abs(pose_diffroll))/num_points
    average_pitch = np.sum(np.abs(pose_diffpitch))/num_points
    average_yaw = np.sum(np.abs(pose_diffyaw))/num_points

    return average_x, average_y, average_z, average_roll, average_pitch, average_yaw


def get_pose_vector(value):
    print("reading file: ", "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/comparison_data_filters/pose_"+changing_parameter +
          str(value)+".csv")
    data = pd.read_csv("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/comparison_data_filters/pose_"+changing_parameter +
                       str(value)+".csv", nrows=1000)
    data_x = pd.DataFrame.to_numpy(data["x"])
    data_y = pd.DataFrame.to_numpy(data["y"])
    data_z = pd.DataFrame.to_numpy(data["z"])
    data_roll = pd.DataFrame.to_numpy(data["roll"])
    data_pitch = pd.DataFrame.to_numpy(data["pitch"])
    data_yaw = pd.DataFrame.to_numpy(data["yaw"])

    return data_x, data_y, data_z, data_roll, data_pitch, data_yaw


def get_step_vector(value):
    data = pd.read_csv("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/comparison_data_filters/steps_"+changing_parameter +
                       str(value)+".csv", nrows=1000)
    data_x = pd.DataFrame.to_numpy(data["x"])
    data_y = pd.DataFrame.to_numpy(data["y"])
    data_z = pd.DataFrame.to_numpy(data["z"])
    data_roll = pd.DataFrame.to_numpy(data["roll"])
    data_pitch = pd.DataFrame.to_numpy(data["pitch"])
    data_yaw = pd.DataFrame.to_numpy(data["yaw"])

    return data_x, data_y, data_z, data_roll, data_pitch, data_yaw


def highlight_min(x):
    criteria = x == x.min()
    return ['background-color: #1cd400' if v else ''
            for v in criteria]


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
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_pose.csv", nrows=1000
    )

    GT_xc = pd.DataFrame.to_numpy(GT_overall["x"])
    GT_yc = pd.DataFrame.to_numpy(GT_overall["y"])
    GT_zc = pd.DataFrame.to_numpy(GT_overall["z"])
    GT_rollc = pd.DataFrame.to_numpy(GT_overall["roll"])
    GT_pitchc = pd.DataFrame.to_numpy(GT_overall["pitch"])
    GT_yawc = pd.DataFrame.to_numpy(GT_overall["yaw"])

    odom_file_pose = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_pose.csv", nrows=1000
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

    writer_pose = pd.ExcelWriter(
        "tables/brisk/BRISK_pose_" + changing_parameter + ".xlsx", engine="xlsxwriter")

    writer_steps = pd.ExcelWriter(
        "tables/filters/Distance_filtering_steps_" + changing_parameter + ".xlsx", engine="xlsxwriter")

    df_pose = pd.DataFrame(index=range(6), columns=range(5))
    df_pose.columns = ["1", "2", "3", "4", "5"]
    df_pose.index = ['err x', 'err y', 'err z',
                     'err roll', 'err pitch', 'err yaw']
    df_pose.columns.name = changing_parameter
    df_pose.style.set_caption("HolaHola").to_excel(
        "tables/filters/Distance_filtering_pose_" + changing_parameter + ".xlsx")

    df_steps = pd.DataFrame(index=range(6), columns=range(5))
    df_steps.columns = ["1", "2", "3", "4", "5"]
    df_steps.index = ['err x', 'err y', 'err z',
                      'err roll', 'err pitch', 'err yaw']
    df_steps.columns.name = changing_parameter

    for i in range(1, 6):
        # save pose average errors
        pose_pr_x, pose_pr_y, pose_pr_z, pose_pr_roll, pose_pr_pitch, pose_pr_yaw = get_pose_vector(
            i)
        pose_avg_err_x, pose_avg_err_y, pose_avg_err_z, pose_avg_err_roll, pose_avg_err_pitch, pose_avg_err_yaw = averager_pose(
            pose_pr_x, pose_pr_y, pose_pr_z, pose_pr_roll, pose_pr_pitch, pose_pr_yaw)
        df_pose.loc["err x", str(i)] = "%.5f" % pose_avg_err_x
        df_pose.loc["err y", str(i)] = "%.5f" % pose_avg_err_y
        df_pose.loc["err z", str(i)] = "%.5f" % pose_avg_err_z
        df_pose.loc["err roll", str(i)] = "%.5f" % pose_avg_err_roll
        df_pose.loc["err pitch", str(i)] = "%.5f" % pose_avg_err_pitch
        df_pose.loc["err yaw", str(i)] = "%.5f" % pose_avg_err_yaw
        # save average step errors
        step_pr_x, step_pr_y, step_pr_z, step_pr_roll, step_pr_pitch, step_pr_yaw = get_step_vector(
            i)

        step_avg_err_x, step_avg_err_y, step_avg_err_z, step_avg_err_roll, step_avg_err_pitch, step_avg_err_yaw = averager_step(
            step_pr_x, step_pr_y, step_pr_z, step_pr_roll, step_pr_pitch, step_pr_yaw)
        df_steps.loc["err x", str(i)] = "%.5f" % step_avg_err_x
        df_steps.loc["err y", str(i)] = "%.5f" % step_avg_err_y
        df_steps.loc["err z", str(i)] = "%.5f" % step_avg_err_z
        df_steps.loc["err roll", str(i)] = "%.5f" % step_avg_err_roll
        df_steps.loc["err pitch", str(i)] = "%.5f" % step_avg_err_pitch
        df_steps.loc["err yaw", str(i)] = "%.5f" % step_avg_err_yaw

    print("We got here!!!!")
    df_pose.to_excel(writer_pose, sheet_name="sheet 1")
    df_steps.to_excel(writer_steps, sheet_name="sheet 1")
    writer_pose.save()
    writer_steps.save()
    df_pose.style.apply(highlight_min, 1).to_excel(
        "tables/filters/Distance_filtering_pose_" + changing_parameter + ".xlsx")
    df_steps.style.apply(highlight_min, 1).to_excel(
        "tables/filters/Distance_filtering_steps_" + changing_parameter + ".xlsx")
    # "KLT_mintracked:"+klt_min_tracked+"_max_detected:" +
    #  klt_max_detected+"_det_qual:"+klt_quality_level+"_block_size:"+klt_block_size+"_min_det_distance:"+klt_min_det_distance+"_epsilon"+klt_epsilon+"_crit_reps:"+klt_criteria_reps+"_opt_size:"+klt_opt_size+"_num_pyr:"+klt_num_pyramids
