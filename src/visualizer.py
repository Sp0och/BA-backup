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

# orb parameters
# brisk parameters

# klt parameters
klt_min_tracked = "40"
klt_quality_level = "0.05"
klt_block_size = "7"
klt_min_klt_distance = "1"

klt_epsilon = "0.03"
klt_criteria_reps = "100"
klt_opt_size = "15"
klt_num_pyramids = "2"


def plot_values(printBool, extractor):
    figc = plt.figure()
    overall_plot_1 = plt.subplot(4, 2, 1)
    plt.title("Total X")
    plt.plot(odom_c_timestamps, odom_xc, 'y', label='Scan to Scan 10 Hz')
    plt.plot(complete_timestamps, prediction_xc, 'b', label='Prediction')
    plt.plot(complete_timestamps, GT_xc, 'g--', label='Ground Truth')
    overall_plot_1.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_xc -
    #          GT_xc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_xc-GT_xc,
    #          'c--', label='error odom - GT')
    # overall_plot_1.legend(
    #     ["error pred - GT\nmean: %.5f" % mean_ep_x+" std: %.5f" % SD_ep_x, "error odom - GT\nmean: %.5f" % mean_eo_x+" std: %.5f" % SD_eo_x], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot_2 = plt.subplot(4, 2, 3)
    plt.title("Total Y")
    plt.plot(odom_c_timestamps, odom_yc, 'y', label='Scan to Scan 10 Hz')
    plt.plot(complete_timestamps, prediction_yc, 'b', label='Prediction')
    plt.plot(complete_timestamps, GT_yc, 'g--', label='Ground Truth')
    overall_plot_2.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_yc -
    #          GT_yc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_yc-GT_yc,
    #          'c--', label='error odom - GT')
    # overall_plot_2.legend(["error pred - GT\nmean: %.5f" % mean_ep_y +
    #                       " std: %.5f" % SD_ep_y, "error odom - GT\nmean: %.5f" % mean_eo_y+" std: %.5f" % SD_eo_y], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot_3 = plt.subplot(4, 2, 5)
    plt.title("Total Z")
    plt.plot(odom_c_timestamps, odom_zc, 'y', label='Scan to Scan 10 Hz')
    plt.plot(complete_timestamps, prediction_zc, 'b', label='Prediction')
    plt.plot(complete_timestamps, GT_zc, 'g--', label='Ground Truth')
    overall_plot_3.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_zc -
    #          GT_zc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_zc-GT_zc,
    #          'c--', label='error odom - GT')
    # overall_plot_3.legend(["error pred - GT\nmean: %.5f" % mean_ep_z +
    #                       " std: %.5f" % SD_ep_z, "error odom - GT\nmean: %.5f" % mean_eo_z+" std: %.5f" % SD_eo_z], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot_4 = plt.subplot(4, 2, 2)
    plt.title("Rot. roll")
    plt.plot(odom_c_timestamps, 57.2858*odom_rollc,
             'y', label='Scan to Scan 10 Hz')
    plt.plot(complete_timestamps, 57.2858 *
             prediction_rollc, 'b', label='Prediction')
    plt.plot(complete_timestamps, 57.2858 *
             GT_rollc, 'g--', label='Ground Truth')
    overall_plot_4.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_rollc -
    #          GT_rollc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_rollc -
    #          GT_rollc, 'c--', label='error odom - GT')
    # overall_plot_4.legend(["error pred - GT\nmean: %.5f" % mean_ep_roll +
    #                       " std: %.5f" % SD_ep_roll, "error odom - GT\nmean: %.5f" % mean_eo_roll+" std: %.5f" % SD_eo_roll], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot_5 = plt.subplot(4, 2, 4)
    plt.title("Rot. pitch")
    plt.plot(odom_c_timestamps, 57.2858*odom_pitchc,
             'y', label='Scan to Scan 10 Hz')
    plt.plot(complete_timestamps, 57.2858 *
             prediction_pitchc, 'b', label='Prediction')
    plt.plot(complete_timestamps, 57.2858 *
             GT_pitchc, 'g--', label='Ground Truth')
    overall_plot_5.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_pitchc -
    #          GT_pitchc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_pitchc -
    #          GT_pitchc, 'c--', label='error odom - GT')
    # overall_plot_5.legend(["error pred - GT\nmean: %.5f" % mean_ep_pitch +
    #                       " std: %.5f" % SD_ep_pitch, "error odom - GT\nmean: %.5f" % mean_eo_pitch+" std: %.5f" % SD_eo_pitch], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot_6 = plt.subplot(4, 2, 6)
    plt.title("Rot. yaw")
    plt.plot(odom_c_timestamps, 57.2858*odom_yawc,
             'y', label='Scan to Scan 10 Hz')
    plt.plot(complete_timestamps, 57.2858 *
             prediction_yawc, 'b', label='Prediction')
    plt.plot(complete_timestamps, 57.2858*GT_yawc, 'g--', label='Ground Truth')
    overall_plot_6.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_yawc -
    #          GT_yawc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_yawc -
    #          GT_yawc, 'c--', label='error odom - GT')
    # overall_plot_6.legend(["error pred - GT\nmean: %.5f" % mean_ep_yaw +
    #                       " std: %.5f" % SD_ep_yaw, "error odom - GT\nmean: %.5f" % mean_eo_yaw+" std: %.5f" % SD_eo_yaw], shadow=True)
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

    figs = plt.figure()

    step_graph_1 = plt.subplot(4, 2, 1)
    plt.title("X")
    plt.plot(odom_s_timestamps, odom_xs, 'y', label='Scan to Scan 10 Hz')
    plt.plot(step_timestamps, prediction_xs, 'b', label='Prediction')
    plt.plot(step_timestamps, GT_xs, 'g--', label='Ground Truth')
    step_graph_1.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_xs -
    #          GT_xs, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_xs-GT_xs, 'c--', label='error odom - GT')
    # step_graph_1.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_x +
    #                     " std: %.5f" % step_SD_ep_x, "error odom - GT\nmean: %.5f" % step_mean_eo_x+" std: %.5f" % step_SD_eo_x], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph_2 = plt.subplot(4, 2, 3)
    plt.title("Y")
    plt.plot(odom_s_timestamps, odom_ys, 'y', label='Scan to Scan 10 Hz')
    plt.plot(step_timestamps, prediction_ys, 'b', label='Prediction')
    plt.plot(step_timestamps, GT_ys, 'g--', label='Ground Truth')
    step_graph_2.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_ys -
    #          GT_ys, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_ys-GT_ys, 'c--', label='error odom - GT')
    # step_graph_2.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_y +
    #                     " std: %.5f" % step_SD_ep_y, "error odom - GT\nmean: %.5f" % step_mean_eo_y+" std: %.5f" % step_SD_eo_y], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph_3 = plt.subplot(4, 2, 5)
    plt.title("Z")
    plt.plot(odom_s_timestamps, odom_zs, 'y', label='Scan to Scan 10 Hz')
    plt.plot(step_timestamps, prediction_zs, 'b', label='Prediction')
    plt.plot(step_timestamps, GT_zs, 'g--', label='Ground Truth')
    step_graph_3.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_zs -
    #          GT_zs, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_zs-GT_zs, 'c--', label='error odom - GT')
    # step_graph_3.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_z +
    #                     " std: %.5f" % step_SD_ep_z, "error odom - GT\nmean: %.5f" % step_mean_eo_z+" std: %.5f" % step_SD_eo_z], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph_4 = plt.subplot(4, 2, 2)
    plt.title("Rot. roll")
    plt.plot(odom_s_timestamps, 57.2858*odom_rolls,
             'y', label='Scan to Scan 10 Hz')
    plt.plot(step_timestamps, 57.2858 *
             prediction_rolls, 'b', label='Prediction')
    plt.plot(step_timestamps, 57.2858 *
             GT_rolls, 'g--', label='Ground Truth')
    step_graph_4.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_rolls -
    #          GT_rolls, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_rolls-GT_rolls,
    #          'c--', label='error odom - GT')
    # step_graph_4.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_roll +
    #                     " std: %.5f" % step_SD_ep_roll, "error odom - GT\nmean: %.5f" % step_mean_eo_roll+" std: %.5f" % step_SD_eo_roll], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph_5 = plt.subplot(4, 2, 4)
    plt.title("Rot. pitch")
    plt.plot(odom_s_timestamps, 57.2858*odom_pitchs,
             'y', label='Scan to Scan 10 Hz')
    plt.plot(step_timestamps, 57.2858 *
             prediction_pitchs, 'b', label='Prediction')
    plt.plot(step_timestamps, 57.2858 *
             GT_pitchs, 'g--', label='Ground Truth')
    step_graph_5.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_pitchs -
    #          GT_pitchs, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_pitchs -
    #          GT_pitchs, 'c--', label='error odom - GT')
    # step_graph_5.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_pitch +
    #                     " std: %.5f" % step_SD_ep_pitch, "error odom - GT\nmean: %.5f" % step_mean_eo_pitch+" std: %.5f" % step_SD_eo_pitch], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph_6 = plt.subplot(4, 2, 6)
    plt.title("Rot. yaw")
    plt.plot(odom_s_timestamps, 57.2858*odom_yaws,
             'y', label='Scan to Scan 10 Hz')
    plt.plot(step_timestamps, 57.2858 *
             prediction_yaws, 'b', label='Prediction')
    plt.plot(step_timestamps, 57.2858*GT_yaws, 'g--', label='Ground Truth')
    step_graph_6.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_yaws -
    #          GT_yaws, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_yaws-GT_yaws,
    #          'c--', label='error odom - GT')
    # step_graph_6.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_yaw +
    #                     " std: %.5f" % step_SD_ep_yaw, "error odom - GT\nmean: %.5f" % step_mean_eo_yaw+" std: %.5f" % step_SD_eo_yaw], shadow=True)
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
    figc.set_figheight(15)
    figs.set_figheight(15)
    figc.set_figwidth(15)
    figs.set_figwidth(15)
    plt.show()
    if(printBool):
        if(extractor == "orb"):
            figc.savefig("pdfs/Path_plot" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
        elif(extractor == "brisk"):
            figc.savefig("pdfs/Path_plot" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
        else:
            figc.savefig("pdfs/Path_plot" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                         "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                         "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')


def plot_errors(printBool, extractor):

    error_prediction_x = np.abs(prediction_xc - GT_xc)
    error_prediction_y = np.abs(prediction_yc - GT_yc)
    error_prediction_z = np.abs(prediction_zc - GT_zc)
    error_prediction_roll = np.abs(57.2858*(prediction_rollc - GT_rollc))
    error_prediction_pitch = np.abs(57.2858*(prediction_pitchc - GT_pitchc))
    error_prediction_yaw = np.abs(57.2858*(prediction_yawc - GT_yawc))

    error_odom_x = np.abs(odom_xc - GT_xc)
    error_odom_y = np.abs(odom_yc - GT_yc)
    error_odom_z = np.abs(odom_zc - GT_zc)
    error_odom_roll = np.abs(57.2858*(odom_rollc - GT_rollc))
    error_odom_pitch = np.abs(57.2858*(odom_pitchc - GT_pitchc))
    error_odom_yaw = np.abs(57.2858*(odom_yawc - GT_yawc))

    mean_ep_x = np.mean(error_prediction_x)
    mean_ep_y = np.mean(error_prediction_y)
    mean_ep_z = np.mean(error_prediction_z)
    mean_ep_roll = np.mean(error_prediction_roll)
    mean_ep_pitch = np.mean(error_prediction_pitch)
    mean_ep_yaw = np.mean(error_prediction_yaw)

    SD_ep_x = np.std(error_prediction_x)
    SD_ep_y = np.std(error_prediction_y)
    SD_ep_z = np.std(error_prediction_z)
    SD_ep_roll = np.std(error_prediction_roll)
    SD_ep_pitch = np.std(error_prediction_pitch)
    SD_ep_yaw = np.std(error_prediction_yaw)

    mean_eo_x = np.mean(error_odom_x)
    mean_eo_y = np.mean(error_odom_y)
    mean_eo_z = np.mean(error_odom_z)
    mean_eo_roll = np.mean(error_odom_roll)
    mean_eo_pitch = np.mean(error_odom_pitch)
    mean_eo_yaw = np.mean(error_odom_yaw)

    SD_eo_x = np.std(error_odom_x)
    SD_eo_y = np.std(error_odom_y)
    SD_eo_z = np.std(error_odom_z)
    SD_eo_roll = np.std(error_odom_roll)
    SD_eo_pitch = np.std(error_odom_pitch)
    SD_eo_yaw = np.std(error_odom_yaw)

    step_error_prediction_x = np.abs(prediction_xs - GT_xs)
    step_error_prediction_y = np.abs(prediction_ys - GT_ys)
    step_error_prediction_z = np.abs(prediction_zs - GT_zs)
    step_error_prediction_roll = np.abs(57.2858*(prediction_rolls - GT_rolls))
    step_error_prediction_pitch = np.abs(
        57.2858*(prediction_pitchs - GT_pitchs))
    step_error_prediction_yaw = np.abs(57.2858*(prediction_yaws - GT_yaws))

    step_error_odom_x = np.abs(odom_xs - GT_xs)
    step_error_odom_y = np.abs(odom_ys - GT_ys)
    step_error_odom_z = np.abs(odom_zs - GT_zs)
    step_error_odom_roll = np.abs(57.2858*(odom_rolls - GT_rolls))
    step_error_odom_pitch = np.abs(57.2858*(odom_pitchs - GT_pitchs))
    step_error_odom_yaw = np.abs(57.2858*(odom_yaws - GT_yaws))

    step_mean_ep_x = np.mean(step_error_prediction_x)
    step_mean_ep_y = np.mean(step_error_prediction_y)
    step_mean_ep_z = np.mean(step_error_prediction_z)
    step_mean_ep_roll = np.mean(step_error_prediction_roll)
    step_mean_ep_pitch = np.mean(step_error_prediction_pitch)
    step_mean_ep_yaw = np.mean(step_error_prediction_yaw)

    step_SD_ep_x = np.std(step_error_prediction_x)
    step_SD_ep_y = np.std(step_error_prediction_y)
    step_SD_ep_z = np.std(step_error_prediction_z)
    step_SD_ep_roll = np.std(step_error_prediction_roll)
    step_SD_ep_pitch = np.std(step_error_prediction_pitch)
    step_SD_ep_yaw = np.std(step_error_prediction_yaw)

    step_mean_eo_x = np.mean(step_error_odom_x)
    step_mean_eo_y = np.mean(step_error_odom_y)
    step_mean_eo_z = np.mean(step_error_odom_z)
    step_mean_eo_roll = np.mean(step_error_odom_roll)
    step_mean_eo_pitch = np.mean(step_error_odom_pitch)
    step_mean_eo_yaw = np.mean(step_error_odom_yaw)

    step_SD_eo_x = np.std(step_error_odom_x)
    step_SD_eo_y = np.std(step_error_odom_y)
    step_SD_eo_z = np.std(step_error_odom_z)
    step_SD_eo_roll = np.std(step_error_odom_roll)
    step_SD_eo_pitch = np.std(step_error_odom_pitch)
    step_SD_eo_yaw = np.std(step_error_odom_yaw)

    figc = plt.figure()
    overall_plot_1 = plt.subplot(4, 2, 1)
    plt.title("Total X")
    plt.plot(complete_timestamps, prediction_xc -
             GT_xc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_xc-GT_xc,
             'c--', label='error odom - GT')
    overall_plot_1.legend(
        ["error pred - GT\nmean: %.5f" % mean_ep_x+" std: %.5f" % SD_ep_x, "error odom - GT\nmean: %.5f" % mean_eo_x+" std: %.5f" % SD_eo_x], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot_2 = plt.subplot(4, 2, 3)
    plt.title("Total Y")
    plt.plot(complete_timestamps, prediction_yc -
             GT_yc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_yc-GT_yc,
             'c--', label='error odom - GT')
    overall_plot_2.legend(["error pred - GT\nmean: %.5f" % mean_ep_y +
                          " std: %.5f" % SD_ep_y, "error odom - GT\nmean: %.5f" % mean_eo_y+" std: %.5f" % SD_eo_y], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot_3 = plt.subplot(4, 2, 5)
    plt.title("Total Z")
    plt.plot(complete_timestamps, prediction_zc -
             GT_zc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_zc-GT_zc,
             'c--', label='error odom - GT')
    overall_plot_3.legend(["error pred - GT\nmean: %.5f" % mean_ep_z +
                          " std: %.5f" % SD_ep_z, "error odom - GT\nmean: %.5f" % mean_eo_z+" std: %.5f" % SD_eo_z], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot_4 = plt.subplot(4, 2, 2)
    plt.title("Rot. roll")
    plt.plot(complete_timestamps, prediction_rollc -
             GT_rollc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_rollc -
             GT_rollc, 'c--', label='error odom - GT')
    overall_plot_4.legend(["error pred - GT\nmean: %.5f" % mean_ep_roll +
                          " std: %.5f" % SD_ep_roll, "error odom - GT\nmean: %.5f" % mean_eo_roll+" std: %.5f" % SD_eo_roll], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot_5 = plt.subplot(4, 2, 4)
    plt.title("Rot. pitch")
    plt.plot(complete_timestamps, prediction_pitchc -
             GT_pitchc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_pitchc -
             GT_pitchc, 'c--', label='error odom - GT')
    overall_plot_5.legend(["error pred - GT\nmean: %.5f" % mean_ep_pitch +
                          " std: %.5f" % SD_ep_pitch, "error odom - GT\nmean: %.5f" % mean_eo_pitch+" std: %.5f" % SD_eo_pitch], shadow=True)
    plt.xlabel('s')
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot_6 = plt.subplot(4, 2, 6)
    plt.title("Rot. yaw")
    plt.plot(complete_timestamps, prediction_yawc -
             GT_yawc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_yawc -
             GT_yawc, 'c--', label='error odom - GT')
    overall_plot_6.legend(["error pred - GT\nmean: %.5f" % mean_ep_yaw +
                          " std: %.5f" % SD_ep_yaw, "error odom - GT\nmean: %.5f" % mean_eo_yaw+" std: %.5f" % SD_eo_yaw], shadow=True)
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

    figs = plt.figure()

    step_graph_1 = plt.subplot(4, 2, 1)
    plt.title("X")
    plt.plot(step_timestamps, prediction_xs -
             GT_xs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_xs-GT_xs, 'c--', label='error odom - GT')
    step_graph_1.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_x +
                        " std: %.5f" % step_SD_ep_x, "error odom - GT\nmean: %.5f" % step_mean_eo_x+" std: %.5f" % step_SD_eo_x], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph_2 = plt.subplot(4, 2, 3)
    plt.title("Y")
    plt.plot(step_timestamps, prediction_ys -
             GT_ys, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_ys-GT_ys, 'c--', label='error odom - GT')
    step_graph_2.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_y +
                        " std: %.5f" % step_SD_ep_y, "error odom - GT\nmean: %.5f" % step_mean_eo_y+" std: %.5f" % step_SD_eo_y], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph_3 = plt.subplot(4, 2, 5)
    plt.title("Z")
    plt.plot(step_timestamps, prediction_zs -
             GT_zs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_zs-GT_zs, 'c--', label='error odom - GT')
    step_graph_3.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_z +
                        " std: %.5f" % step_SD_ep_z, "error odom - GT\nmean: %.5f" % step_mean_eo_z+" std: %.5f" % step_SD_eo_z], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph_4 = plt.subplot(4, 2, 2)
    plt.title("Rot. roll")
    plt.plot(step_timestamps, prediction_rolls -
             GT_rolls, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_rolls-GT_rolls,
             'c--', label='error odom - GT')
    step_graph_4.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_roll +
                        " std: %.5f" % step_SD_ep_roll, "error odom - GT\nmean: %.5f" % step_mean_eo_roll+" std: %.5f" % step_SD_eo_roll], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph_5 = plt.subplot(4, 2, 4)
    plt.title("Rot. pitch")
    plt.plot(step_timestamps, prediction_pitchs -
             GT_pitchs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_pitchs -
             GT_pitchs, 'c--', label='error odom - GT')
    step_graph_5.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_pitch +
                        " std: %.5f" % step_SD_ep_pitch, "error odom - GT\nmean: %.5f" % step_mean_eo_pitch+" std: %.5f" % step_SD_eo_pitch], shadow=True)
    plt.xlabel('s')
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph_6 = plt.subplot(4, 2, 6)
    plt.title("Rot. yaw")
    plt.plot(step_timestamps, prediction_yaws -
             GT_yaws, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_yaws-GT_yaws,
             'c--', label='error odom - GT')
    step_graph_6.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_yaw +
                        " std: %.5f" % step_SD_ep_yaw, "error odom - GT\nmean: %.5f" % step_mean_eo_yaw+" std: %.5f" % step_SD_eo_yaw], shadow=True)
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
    figc.set_figheight(15)
    figs.set_figheight(15)
    figc.set_figwidth(15)
    figs.set_figwidth(15)
    plt.show()
    if(printBool):
        if(extractor == "orb"):
            figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
        elif(extractor == "brisk"):
            figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + ".pdf", bbox_inches='tight')
        else:
            figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                         "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                         min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                         "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')


def plot_trajectory(printBool, extractor):
    path_figure = plt.figure()
    pathplot = plt.subplot(1, 1, 1)
    plt.title("Path Plot")
    plt.plot(prediction_xc, prediction_yc, 'b', label='x vs y Prediction')
    plt.plot(GT_xc, GT_yc, 'g', label='x vs y GT')
    plt.plot(odom_xc, odom_yc, 'y', label='x vs y Odom')
    plt.ylabel('y')
    plt.xlabel('x')
    pathplot.legend(shadow=True)
    path_figure.set_figheight(15)
    path_figure.set_figwidth(15)
    plt.show()
    if(printBool):
        if(extractor == "orb"):
            path_figure.savefig("pdfs/Trajectory_plot_vs_Scan2Scan" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                                min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                                "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                                "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                                "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')
        elif(extractor == "brisk"):
            path_figure.savefig("pdfs/Trajectory_plot_vs_Scan2Scan" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                                min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                                "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                                "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                                "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')
        else:
            path_figure.savefig("pdfs/Trajectory_plot_vs_Scan2Scan" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                                min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smoothing:" + smoothing+"_length:" + length +
                                "_minTracked:"+klt_min_tracked + "_detectQuality:"+klt_quality_level + "_detectBlockSize:"+klt_block_size +
                                "_minDetectDistance:"+klt_min_klt_distance+"_matchErrorTresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                                "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')


if __name__ == "__main__":

    prediction_step = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/prediction_steps.csv", nrows=1000)
    prediction_xs = pd.DataFrame.to_numpy(prediction_step["x"])
    prediction_ys = pd.DataFrame.to_numpy(prediction_step["y"])
    prediction_zs = pd.DataFrame.to_numpy(prediction_step["z"])
    prediction_rolls = pd.DataFrame.to_numpy(prediction_step["roll"])
    prediction_pitchs = pd.DataFrame.to_numpy(prediction_step["pitch"])
    prediction_yaws = pd.DataFrame.to_numpy(prediction_step["yaw"])

    prediction_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/comparison_data/pose_opt_size15.csv", nrows=1000)
    prediction_xc = pd.DataFrame.to_numpy(prediction_overall["x"])
    prediction_yc = pd.DataFrame.to_numpy(prediction_overall["y"])
    prediction_zc = pd.DataFrame.to_numpy(prediction_overall["z"])
    prediction_rollc = pd.DataFrame.to_numpy(prediction_overall["roll"])
    prediction_pitchc = pd.DataFrame.to_numpy(prediction_overall["pitch"])
    prediction_yawc = pd.DataFrame.to_numpy(prediction_overall["yaw"])

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

    feature_number_file = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/feature_number.csv", nrows=1000
    )

    feature_number = pd.DataFrame.to_numpy(
        feature_number_file["num_of_features"])

    # create timestamps

    complete_timestamps = pd.DataFrame.to_numpy(
        prediction_overall["time"])
    step_timestamps = pd.DataFrame.to_numpy(prediction_step["time"])
    odom_s_timestamps = pd.DataFrame.to_numpy(odom_file_steps["time"])
    odom_c_timestamps = pd.DataFrame.to_numpy(odom_file_complete["time"])

    feature_timestamps = pd.DataFrame.to_numpy(feature_number_file["time"])
    feature_timestamps = feature_timestamps - complete_timestamps[0]
    step_timestamps = step_timestamps - complete_timestamps[0]
    odom_s_timestamps = odom_s_timestamps / 1e09
    odom_s_timestamps = odom_s_timestamps - complete_timestamps[0]
    odom_c_timestamps = odom_c_timestamps / 1e09
    odom_c_timestamps = odom_c_timestamps - complete_timestamps[0]
    complete_timestamps = complete_timestamps - complete_timestamps[0]

    plot_values(False, "klt")
    plot_errors(False, "klt")
    plot_trajectory(False, "klt")

    # calculation of average error:
    # diffx = prediction_xs-GT_xs
    # diffy = prediction_ys-GT_ys
    # diffz = prediction_zs-GT_zs
    # diffroll = prediction_rolls-GT_rolls
    # diffpitch = prediction_pitchs-GT_pitchs
    # diffyaw = prediction_yaws-GT_yaws

    # average_x = np.sum(np.abs(diffx))/1000
    # average_y = np.sum(np.abs(diffy))/1000
    # average_z = np.sum(np.abs(diffz))/1000
    # average_roll = np.sum(np.abs(diffroll))/1000
    # average_pitch = np.sum(np.abs(diffpitch))/1000
    # average_yaw = np.sum(np.abs(diffyaw))/1000

    # print("Average diff x: ", average_x)
    # print("Average diff y: ", average_y)
    # print("Average diff z: ", average_z)
    # print("Average diff roll: ", average_roll)
    # print("Average diff pitch: ", average_pitch)
    # print("Average diff yaw: ", average_yaw)

    # open the old wrong files for a comparison:

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

    # plot the old plots:
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
