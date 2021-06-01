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

    prediction_timestamps = pd.DataFrame.to_numpy(
        prediction_step["time"])
    GT_timestamps = pd.DataFrame.to_numpy(GT_steps["time"])

    GT_timestamps = GT_timestamps / 10**9

    time_shift = GT_timestamps - prediction_timestamps
    GT_timestamps = GT_timestamps - time_shift
    GT_timestamps = GT_timestamps - prediction_timestamps[0]
    prediction_timestamps = prediction_timestamps - prediction_timestamps[0]

    # create the plots

    plt.figure()
    overall_plot = plt.subplot(3, 2, 1)
    plt.title("Translation in x")
    plt.plot(prediction_timestamps, prediction_xc, 'b', label='Prediction')
    plt.plot(GT_timestamps, GT_xc, 'g', label='Ground Truth')
    # plt.plot(x, prediction_xc-GT_xc, 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('overall change translation in x [m]')

    overall_plot = plt.subplot(3, 2, 3)
    plt.title("Translation in y")
    plt.plot(prediction_timestamps, prediction_yc, 'b', label='Prediction')
    plt.plot(GT_timestamps, GT_yc, 'g', label='Ground Truth')
    # plt.plot(x, prediction_yc-GT_yc, 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('overall change translation in y [m]')

    overall_plot = plt.subplot(3, 2, 5)
    plt.title("Translation in z")
    plt.plot(prediction_timestamps, prediction_zc, 'b', label='Prediction')
    plt.plot(GT_timestamps, GT_zc, 'g', label='Ground Truth')
    # plt.plot(x, prediction_zc-GT_zc, 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('overall change translation in z [m]')

    overall_plot = plt.subplot(3, 2, 2)
    plt.title("rotation roll")
    plt.plot(prediction_timestamps, 57.2858 *
             prediction_rollc, 'b', label='Prediction')
    plt.plot(GT_timestamps, 57.2858*GT_rollc, 'g', label='Ground Truth')
    # plt.plot(x, 57.2858*(prediction_rollc-GT_rollc), 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('overall change row angle [°]')

    overall_plot = plt.subplot(3, 2, 4)
    plt.title("Rotation pitch")
    plt.plot(prediction_timestamps, 57.2858 *
             prediction_pitchc, 'b', label='Prediction')
    plt.plot(GT_timestamps, 57.2858*GT_pitchc, 'g', label='Ground Truth')
    # plt.plot(x, 57.2858*(prediction_pitchc-GT_pitchc), 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('overall change pitch angle [°]')

    overall_plot = plt.subplot(3, 2, 6)
    plt.title("Rotation yaw")
    plt.plot(prediction_timestamps, 57.2858 *
             prediction_yawc, 'b', label='Prediction')
    plt.plot(GT_timestamps, 57.2858*GT_yawc, 'g', label='Ground Truth')
    # plt.plot(x, 57.2858*(prediction_yawc-GT_yawc), 'r', label='error')
    overall_plot.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('overall change yaw angle [°]')

    plt.figure()

    step_graph = plt.subplot(3, 2, 1)
    plt.title("Translation in x")
    plt.plot(prediction_timestamps, prediction_xs, 'b', label='Prediction')
    plt.plot(GT_timestamps, GT_xs, 'g', label='Ground Truth')
    # plt.plot(x, prediction_xs-GT_xs, 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('step change translation in x [m]')

    step_graph = plt.subplot(3, 2, 3)
    plt.title("Translation in y")
    plt.plot(prediction_timestamps, prediction_ys, 'b', label='Prediction')
    plt.plot(GT_timestamps, GT_ys, 'g', label='Ground Truth')
    # plt.plot(x, prediction_ys-GT_ys, 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('step change translation in y [m]')

    step_graph = plt.subplot(3, 2, 5)
    plt.title("Translation in z")
    plt.plot(prediction_timestamps, prediction_zs, 'b', label='Prediction')
    plt.plot(GT_timestamps, GT_zs, 'g', label='Ground Truth')
    # plt.plot(x, prediction_zs-GT_zs, 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('step change translation in z [m]')

    step_graph = plt.subplot(3, 2, 2)
    plt.title("rotation roll")
    plt.plot(prediction_timestamps, 57.2858 *
             prediction_rolls, 'b', label='Prediction')
    plt.plot(GT_timestamps, 57.2858*GT_rolls, 'g', label='Ground Truth')
    # plt.plot(x, 57.2858*(prediction_rolls-GT_rolls), 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('step change row angle [°]')

    step_graph = plt.subplot(3, 2, 4)
    plt.title("Rotation pitch")
    plt.plot(prediction_timestamps, 57.2858 *
             prediction_pitchs, 'b', label='Prediction')
    plt.plot(GT_timestamps, 57.2858*GT_pitchs, 'g', label='Ground Truth')
    # plt.plot(x, 57.2858*(prediction_pitchs-GT_pitchs), 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('step change pitch angle [°]')

    step_graph = plt.subplot(3, 2, 6)
    plt.title("Rotation yaw")
    plt.plot(prediction_timestamps, 57.2858 *
             prediction_yaws, 'b', label='Prediction')
    plt.plot(GT_timestamps, 57.2858*GT_yaws, 'g', label='Ground Truth')
    # plt.plot(x, 57.2858*(prediction_yaws-GT_yaws), 'r', label='error')
    step_graph.legend(shadow=True)
    plt.xlabel('steps')
    plt.ylabel('step change yaw angle [°]')

    plt.show()
