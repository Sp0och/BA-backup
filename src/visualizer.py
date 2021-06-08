import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if __name__ == "__main__":

    prediction_step = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/prediction_steps.csv", nrows=1000)
    # Create arrays from file
    prediction_xs = pd.DataFrame.to_numpy(prediction_step["x"])
    prediction_ys = pd.DataFrame.to_numpy(prediction_step["y"])
    prediction_zs = pd.DataFrame.to_numpy(prediction_step["z"])
    prediction_rolls = pd.DataFrame.to_numpy(prediction_step["roll"])
    prediction_pitchs = pd.DataFrame.to_numpy(prediction_step["pitch"])
    prediction_yaws = pd.DataFrame.to_numpy(prediction_step["yaw"])

    prediction_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/prediction_complete.csv", nrows=1000)
    # Create arrays from file
    prediction_xc = pd.DataFrame.to_numpy(prediction_overall["x"])
    prediction_yc = pd.DataFrame.to_numpy(prediction_overall["y"])
    prediction_zc = pd.DataFrame.to_numpy(prediction_overall["z"])
    prediction_rollc = pd.DataFrame.to_numpy(prediction_overall["roll"])
    prediction_pitchc = pd.DataFrame.to_numpy(prediction_overall["pitch"])
    prediction_yawc = pd.DataFrame.to_numpy(prediction_overall["yaw"])

    GT_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_steps.csv", nrows=1000
    )

    # # Create arrays from file
    GT_xs = pd.DataFrame.to_numpy(GT_steps["x"])
    GT_ys = pd.DataFrame.to_numpy(GT_steps["y"])
    GT_zs = pd.DataFrame.to_numpy(GT_steps["z"])
    GT_rolls = pd.DataFrame.to_numpy(GT_steps["roll"])
    GT_pitchs = pd.DataFrame.to_numpy(GT_steps["pitch"])
    GT_yaws = pd.DataFrame.to_numpy(GT_steps["yaw"])

    GT_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_complete.csv", nrows=1000
    )

    # Create arrays from file
    GT_xc = pd.DataFrame.to_numpy(GT_overall["x"])
    GT_yc = pd.DataFrame.to_numpy(GT_overall["y"])
    GT_zc = pd.DataFrame.to_numpy(GT_overall["z"])
    GT_rollc = pd.DataFrame.to_numpy(GT_overall["roll"])
    GT_pitchc = pd.DataFrame.to_numpy(GT_overall["pitch"])
    GT_yawc = pd.DataFrame.to_numpy(GT_overall["yaw"])

    feature_number_file = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/feature_number.csv", nrows=1000
    )

    feature_number = pd.DataFrame.to_numpy(
        feature_number_file["num_of_features"])
    complete_timestamps = pd.DataFrame.to_numpy(
        prediction_overall["time"])
    step_timestamps = pd.DataFrame.to_numpy(prediction_step["time"])

    step_timestamps = step_timestamps - complete_timestamps[0]
    feature_timestamps = pd.DataFrame.to_numpy(feature_number_file["time"])
    feature_timestamps = feature_timestamps - complete_timestamps[0]
    print("feature timestamps: ", feature_timestamps)
    complete_timestamps = complete_timestamps - complete_timestamps[0]

    # create the plots
    plt.figure()
    overall_plot = plt.subplot(4, 2, 1)
    plt.title("Total X")
    # plt.plot(complete_timestamps, prediction_xc, 'b', label='Prediction')
    # plt.plot(complete_timestamps, GT_xc, 'g', label='Ground Truth')
    plt.plot(complete_timestamps, prediction_xc-GT_xc, 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot = plt.subplot(4, 2, 3)
    plt.title("Total Y")
    # plt.plot(complete_timestamps, prediction_yc, 'b', label='Prediction')
    # plt.plot(complete_timestamps, GT_yc, 'g', label='Ground Truth')
    plt.plot(complete_timestamps, prediction_yc-GT_yc, 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot = plt.subplot(4, 2, 5)
    plt.title("Total Z")
    # plt.plot(complete_timestamps, prediction_zc, 'b', label='Prediction')
    # plt.plot(complete_timestamps, GT_zc, 'g', label='Ground Truth')
    plt.plot(complete_timestamps, prediction_zc-GT_zc, 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot = plt.subplot(4, 2, 2)
    plt.title("Rot. roll")
    # plt.plot(complete_timestamps, 57.2858 *
    #          prediction_rollc, 'b', label='Prediction')
    # plt.plot(complete_timestamps, 57.2858 *
    #          GT_rollc, 'g', label='Ground Truth')
    plt.plot(complete_timestamps, 57.2858 *
             (prediction_rollc-GT_rollc), 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot = plt.subplot(4, 2, 4)
    plt.title("Rot. pitch")
    # plt.plot(complete_timestamps, 57.2858 *
    #          prediction_pitchc, 'b', label='Prediction')
    # plt.plot(complete_timestamps, 57.2858 *
    #          GT_pitchc, 'g', label='Ground Truth')
    plt.plot(complete_timestamps, 57.2858 *
             (prediction_pitchc-GT_pitchc), 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot = plt.subplot(4, 2, 6)
    plt.title("Rot. yaw")
    # plt.plot(complete_timestamps, 57.2858 *
    #          prediction_yawc, 'b', label='Prediction')
    # plt.plot(complete_timestamps, 57.2858*GT_yawc, 'g', label='Ground Truth')
    plt.plot(complete_timestamps, 57.2858 *
             (prediction_yawc-GT_yawc), 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total yaw angle [°]')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 7)
    plt.title("Features")
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    feature_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 8)
    plt.title("Features")
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    feature_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    plt.figure()

    step_graph = plt.subplot(4, 2, 1)
    plt.title("X")
    # plt.plot(step_timestamps, prediction_xs, 'b', label='Prediction')
    # plt.plot(step_timestamps, GT_xs, 'g', label='Ground Truth')
    plt.plot(step_timestamps, prediction_xs-GT_xs, 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph = plt.subplot(4, 2, 3)
    plt.title("Y")
    # plt.plot(step_timestamps, prediction_ys, 'b', label='Prediction')
    # plt.plot(step_timestamps, GT_ys, 'g', label='Ground Truth')
    plt.plot(step_timestamps, prediction_ys-GT_ys, 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph = plt.subplot(4, 2, 5)
    plt.title("Z")
    # plt.plot(step_timestamps, prediction_zs, 'b', label='Prediction')
    # plt.plot(step_timestamps, GT_zs, 'g', label='Ground Truth')
    plt.plot(step_timestamps, prediction_zs-GT_zs, 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph = plt.subplot(4, 2, 2)
    plt.title("Rot. roll")
    # plt.plot(step_timestamps, 57.2858 *
    #          prediction_rolls, 'b', label='Prediction')
    # plt.plot(step_timestamps, 57.2858 *
    #          GT_rolls, 'g', label='Ground Truth')
    plt.plot(step_timestamps, 57.2858 *
             (prediction_rolls-GT_rolls), 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph = plt.subplot(4, 2, 4)
    plt.title("Rot. pitch")
    # plt.plot(step_timestamps, 57.2858 *
    #          prediction_pitchs, 'b', label='Prediction')
    # plt.plot(step_timestamps, 57.2858 *
    #          GT_pitchs, 'g', label='Ground Truth')
    plt.plot(step_timestamps, 57.2858 *
             (prediction_pitchs-GT_pitchs), 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph = plt.subplot(4, 2, 6)
    plt.title("Rot. yaw")
    # plt.plot(step_timestamps, 57.2858 *
    #          prediction_yaws, 'b', label='Prediction')
    # plt.plot(step_timestamps, 57.2858*GT_yaws, 'g', label='Ground Truth')
    plt.plot(step_timestamps, 57.2858 *
             (prediction_yaws-GT_yaws), 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('step yaw angle [°]')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 7)
    plt.title("Features")
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    feature_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 8)
    plt.title("Features")
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    feature_plot.legend(shadow=True)
    plt.xlabel('s')
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    plt.show()

    OLD_GT_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/OLD_GT_complete.csv", nrows=1000
    )

    # Create arrays from file
    OLD_GT_xc = pd.DataFrame.to_numpy(OLD_GT_overall["x"])
    OLD_GT_yc = pd.DataFrame.to_numpy(OLD_GT_overall["y"])
    OLD_GT_zc = pd.DataFrame.to_numpy(OLD_GT_overall["z"])
    OLD_GT_rollc = pd.DataFrame.to_numpy(OLD_GT_overall["roll"])
    OLD_GT_pitchc = pd.DataFrame.to_numpy(OLD_GT_overall["pitch"])
    OLD_GT_yawc = pd.DataFrame.to_numpy(OLD_GT_overall["yaw"])

    OLD_prediction_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/OLD_prediction_complete.csv", nrows=1000)

    OLD_prediction_xc = pd.DataFrame.to_numpy(OLD_prediction_overall["x"])
    OLD_prediction_yc = pd.DataFrame.to_numpy(OLD_prediction_overall["y"])
    OLD_prediction_zc = pd.DataFrame.to_numpy(OLD_prediction_overall["z"])
    OLD_prediction_rollc = pd.DataFrame.to_numpy(
        OLD_prediction_overall["roll"])
    OLD_prediction_pitchc = pd.DataFrame.to_numpy(
        OLD_prediction_overall["pitch"])
    OLD_prediction_yawc = pd.DataFrame.to_numpy(OLD_prediction_overall["yaw"])

    # set linear x axis
    x = np.linspace(0, 1000, 1000)
    # create the plots

    # sub = plt.subplot(3, 2, 1)
    # plt.title("Translation in x")
    # plt.plot(x, OLD_prediction_xc, 'r')
    # plt.plot(x, OLD_GT_xc, 'b')
    # # plt.xlabel('time')
    # plt.ylabel('overall translation in x')

    # sub = plt.subplot(3, 2, 3)
    # plt.title("Translation in y")
    # plt.plot(x, OLD_prediction_yc, 'r')
    # plt.plot(x, OLD_GT_yc, 'b')
    # # plt.xlabel('time')
    # plt.ylabel('overall translation in y')

    # sub = plt.subplot(3, 2, 5)
    # plt.title("Translation in z")
    # plt.plot(x, OLD_prediction_zc, 'r')
    # plt.plot(x, OLD_GT_zc, 'b')
    # # plt.xlabel('time')
    # plt.ylabel('overall translation in z')

    # sub = plt.subplot(3, 2, 2)
    # plt.title("rotation roll")
    # plt.plot(x, OLD_prediction_rollc, 'r')
    # plt.plot(x, OLD_GT_rollc, 'b')
    # # plt.xlabel('time')
    # plt.ylabel('overall row angle')
    # plt.ylim([-5, 5])

    # sub = plt.subplot(3, 2, 4)
    # plt.title("Rotation pitch")
    # plt.plot(x, OLD_prediction_pitchc, 'r')
    # plt.plot(x, OLD_GT_pitchc, 'b')
    # # plt.xlabel('time')
    # plt.ylabel('overall pitch angle')
    # plt.ylim([-5, 5])

    # sub = plt.subplot(3, 2, 6)
    # plt.title("Rotation yaw")
    # plt.plot(x, OLD_prediction_yawc, 'r')
    # plt.plot(x, OLD_GT_yawc, 'b')
    # # plt.xlabel('time')
    # plt.ylabel('overall yaw angle')
    # plt.ylim([-5, 5])

    # plt.show()
