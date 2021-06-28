import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

file_name = "ambient_0.3_3_2"
directory = "output"

Data = "Intensity"
Extractor = "KLT"
MASK = "ON"
duplicate_size = "1"
max_match_distance = "0.3m"
min_distance = "0.1m"
max_cos = "0.0"
smoothing = "1"
length = "150"
file_length = 11500
# orb parameters
orb_max_features = "1000"
orb_accuracy = "31"
orb_scale_factor = "1.7"
orb_levels = "8"
# brisk parameters
brisk_threshold = "60"
brisk_octaves = "3"
brisk_pattern_scale = "1.0"
# klt parameters
klt_min_tracked = "70"
klt_quality_level = "0.05"
klt_block_size = "7"
klt_min_klt_distance = "1"

klt_epsilon = "0.03"
klt_criteria_reps = "100"
klt_opt_size = "15"
klt_num_pyramids = "2"


def plot_values(printBool, extractor):
    figc = plt.figure()
    overall_plot_1 = plt.subplot(3, 2, 1)
    plt.title("X")
    plt.plot(odom_c_timestamps, odom_xc, 'y', label='LOAM S2S')
    plt.plot(complete_timestamps, prediction_xc, 'b', label='My Method')
    plt.plot(complete_timestamps, GT_xc, 'g', label='Lio-Sam GT')
    # overall_plot_1.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_xc -
    #          GT_xc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_xc-GT_xc,
    #          'c--', label='error odom - GT')
    # overall_plot_1.legend(
    #     ["error pred - GT\nmean: %.5f" % mean_ep_x+" std: %.5f" % SD_ep_x, "error odom - GT\nmean: %.5f" % mean_eo_x+" std: %.5f" % SD_eo_x], shadow=True)
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot_2 = plt.subplot(3, 2, 3)
    plt.title("Y")
    plt.plot(odom_c_timestamps, odom_yc, 'y', label='LOAM S2S')
    plt.plot(complete_timestamps, prediction_yc, 'b', label='My Method')
    plt.plot(complete_timestamps, GT_yc, 'g', label='Lio-Sam GT')
    # overall_plot_2.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_yc -
    #          GT_yc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_yc-GT_yc,
    #          'c--', label='error odom - GT')
    # overall_plot_2.legend(["error pred - GT\nmean: %.5f" % mean_ep_y +
    #                       " std: %.5f" % SD_ep_y, "error odom - GT\nmean: %.5f" % mean_eo_y+" std: %.5f" % SD_eo_y], shadow=True)
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot_3 = plt.subplot(3, 2, 5)
    plt.title("Z")
    plt.plot(odom_c_timestamps, odom_zc, 'y', label='LOAM S2S')
    plt.plot(complete_timestamps, prediction_zc, 'b', label='My Method')
    plt.plot(complete_timestamps, GT_zc, 'g', label='Lio-Sam GT')
    # overall_plot_3.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_zc -
    #          GT_zc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_zc-GT_zc,
    #          'c--', label='error odom - GT')
    # overall_plot_3.legend(["error pred - GT\nmean: %.5f" % mean_ep_z +
    #                       " std: %.5f" % SD_ep_z, "error odom - GT\nmean: %.5f" % mean_eo_z+" std: %.5f" % SD_eo_z], shadow=True)
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot_4 = plt.subplot(3, 2, 2)
    plt.title("Roll")
    plt.plot(odom_c_timestamps, 57.2858*odom_rollc,
             'y', label='LOAM S2S')
    plt.plot(complete_timestamps, 57.2858 *
             prediction_rollc, 'b', label='My Method')
    plt.plot(complete_timestamps, 57.2858 *
             GT_rollc, 'g', label='Lio-Sam GT')
    # overall_plot_4.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_rollc -
    #          GT_rollc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_rollc -
    #          GT_rollc, 'c--', label='error odom - GT')
    # overall_plot_4.legend(["error pred - GT\nmean: %.5f" % mean_ep_roll +
    #                       " std: %.5f" % SD_ep_roll, "error odom - GT\nmean: %.5f" % mean_eo_roll+" std: %.5f" % SD_eo_roll], shadow=True)
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot_5 = plt.subplot(3, 2, 4)
    plt.title("Pitch")
    plt.plot(odom_c_timestamps, 57.2858*odom_pitchc,
             'y', label='LOAM S2S')
    plt.plot(complete_timestamps, 57.2858 *
             prediction_pitchc, 'b', label='My Method')
    plt.plot(complete_timestamps, 57.2858 *
             GT_pitchc, 'g', label='Lio-Sam GT')
    # overall_plot_5.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_pitchc -
    #          GT_pitchc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_pitchc -
    #          GT_pitchc, 'c--', label='error odom - GT')
    # overall_plot_5.legend(["error pred - GT\nmean: %.5f" % mean_ep_pitch +
    #                       " std: %.5f" % SD_ep_pitch, "error odom - GT\nmean: %.5f" % mean_eo_pitch+" std: %.5f" % SD_eo_pitch], shadow=True)
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot_6 = plt.subplot(3, 2, 6)
    plt.title("Yaw")
    plt.plot(odom_c_timestamps, 57.2858*odom_yawc,
             'y', label='LOAM S2S')
    plt.plot(complete_timestamps, 57.2858 *
             prediction_yawc, 'b', label='My Method')
    plt.plot(complete_timestamps, 57.2858*GT_yawc, 'g', label='Lio-Sam GT')
    # overall_plot_6.legend(shadow=True)
    # plt.plot(complete_timestamps, prediction_yawc -
    #          GT_yawc, 'r', label='error pred - GT')
    # plt.plot(complete_timestamps, odom_yawc -
    #          GT_yawc, 'c--', label='error odom - GT')
    # overall_plot_6.legend(["error pred - GT\nmean: %.5f" % mean_ep_yaw +
    #                       " std: %.5f" % SD_ep_yaw, "error odom - GT\nmean: %.5f" % mean_eo_yaw+" std: %.5f" % SD_eo_yaw], shadow=True)
    plt.ylabel('Total yaw angle [°]')
    plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 7)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 8)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    figs = plt.figure()

    step_graph_1 = plt.subplot(3, 2, 1)
    plt.title("X")
    plt.plot(odom_s_timestamps, odom_xs, 'y', label='LOAM S2S')
    plt.plot(step_timestamps, prediction_xs, 'b--', label='My Method')
    plt.plot(step_timestamps, GT_xs, 'g--', label='Lio-Sam GT')
    # step_graph_1.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_xs -
    #          GT_xs, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_xs-GT_xs, 'c--', label='error odom - GT')
    # step_graph_1.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_x +
    #                     " std: %.5f" % step_SD_ep_x, "error odom - GT\nmean: %.5f" % step_mean_eo_x+" std: %.5f" % step_SD_eo_x], shadow=True)
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph_2 = plt.subplot(3, 2, 3)
    plt.title("Y")
    plt.plot(odom_s_timestamps, odom_ys, 'y', label='LOAM S2S')
    plt.plot(step_timestamps, prediction_ys, 'b--', label='My Method')
    plt.plot(step_timestamps, GT_ys, 'g--', label='Lio-Sam GT')
    # step_graph_2.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_ys -
    #          GT_ys, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_ys-GT_ys, 'c--', label='error odom - GT')
    # step_graph_2.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_y +
    #                     " std: %.5f" % step_SD_ep_y, "error odom - GT\nmean: %.5f" % step_mean_eo_y+" std: %.5f" % step_SD_eo_y], shadow=True)
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph_3 = plt.subplot(3, 2, 5)
    plt.title("Z")
    plt.plot(odom_s_timestamps, odom_zs, 'y', label='LOAM S2S')
    plt.plot(step_timestamps, prediction_zs, 'b--', label='My Method')
    plt.plot(step_timestamps, GT_zs, 'g--', label='Lio-Sam GT')
    # step_graph_3.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_zs -
    #          GT_zs, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_zs-GT_zs, 'c--', label='error odom - GT')
    # step_graph_3.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_z +
    #                     " std: %.5f" % step_SD_ep_z, "error odom - GT\nmean: %.5f" % step_mean_eo_z+" std: %.5f" % step_SD_eo_z], shadow=True)
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph_4 = plt.subplot(3, 2, 2)
    plt.title("Roll")
    plt.plot(step_timestamps, 57.2858 *
             GT_rolls, 'g--', label='Lio-Sam GT')
    plt.plot(step_timestamps, 57.2858 *
             prediction_rolls, 'b--', label='My Method')
    plt.plot(odom_s_timestamps, 57.2858*odom_rolls,
             'y', label='LOAM S2S')
    # step_graph_4.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_rolls -
    #          GT_rolls, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_rolls-GT_rolls,
    #          'c--', label='error odom - GT')
    # step_graph_4.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_roll +
    #                     " std: %.5f" % step_SD_ep_roll, "error odom - GT\nmean: %.5f" % step_mean_eo_roll+" std: %.5f" % step_SD_eo_roll], shadow=True)
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph_5 = plt.subplot(3, 2, 4)
    plt.title("Pitch")
    plt.plot(step_timestamps, 57.2858 *
             GT_pitchs, 'g--', label='Lio-Sam GT')
    plt.plot(step_timestamps, 57.2858 *
             prediction_pitchs, 'b--', label='My Method')
    plt.plot(odom_s_timestamps, 57.2858*odom_pitchs,
             'y', label='LOAM S2S')
    # step_graph_5.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_pitchs -
    #          GT_pitchs, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_pitchs -
    #          GT_pitchs, 'c--', label='error odom - GT')
    # step_graph_5.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_pitch +
    #                     " std: %.5f" % step_SD_ep_pitch, "error odom - GT\nmean: %.5f" % step_mean_eo_pitch+" std: %.5f" % step_SD_eo_pitch], shadow=True)
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph_6 = plt.subplot(3, 2, 6)
    plt.title("Yaw")
    plt.plot(step_timestamps, 57.2858*GT_yaws, 'g--', label='Lio-Sam GT')
    plt.plot(step_timestamps, 57.2858 *
             prediction_yaws, 'b--', label='My Method')
    plt.plot(odom_s_timestamps, 57.2858*odom_yaws,
             'y', label='LOAM S2S')
    # step_graph_6.legend(shadow=True)
    # plt.plot(step_timestamps, prediction_yaws -
    #          GT_yaws, 'r', label='error pred - GT')
    # plt.plot(step_timestamps, odom_yaws-GT_yaws,
    #          'c--', label='error odom - GT')
    # step_graph_6.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_yaw +
    #                     " std: %.5f" % step_SD_ep_yaw, "error odom - GT\nmean: %.5f" % step_mean_eo_yaw+" std: %.5f" % step_SD_eo_yaw], shadow=True)
    plt.ylabel('step yaw angle [°]')
    plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 7)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 8)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    figc.set_figheight(15)
    figs.set_figheight(15)
    figc.set_figwidth(15)
    figs.set_figwidth(15)
    plt.show()
    if(printBool):
        if(extractor == "orb"):
            figc.savefig("pdfs/Pose" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" +
                         length + "max_orb_feat:" + orb_max_features +
                         "orb_acc:" + orb_accuracy + "orb_Sc_Fa:"
                         + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" +
                         length + "max_orb_feat:" + orb_max_features+"orb_acc:" + orb_accuracy + "orb_Sc_Fa:" +
                         orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
        elif(extractor == "brisk"):
            figc.savefig("pdfs/Pose" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_P_S:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_P_S:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
        else:
            figc.savefig("pdfs/Pose" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_det_Quality:"+klt_quality_level + "_block_size:"+klt_block_size +
                         "_min_Det_Dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_det_Quality:"+klt_quality_level + "_block_size:"+klt_block_size +
                         "_min_Det_Dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')


def plot_errors(printBool, show_errors, extractor):

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
    overall_plot_1 = plt.subplot(3, 2, 1)
    plt.title("X")
    plt.plot(complete_timestamps, prediction_xc -
             GT_xc, 'r', label='error My Method vs Lio-Sam GT')
    plt.plot(complete_timestamps, odom_xc-GT_xc,
             'c--', label='error LOAM S2S vs  - Lio-Sam GT')
    # if(show_errors):
    #     overall_plot_1.legend(
    #         ["error pred - GT\nmean: %.5f" % mean_ep_x+" std: %.5f" % SD_ep_x, "error odom - GT\nmean: %.5f" % mean_eo_x+" std: %.5f" % SD_eo_x], shadow=True)
    # else:
    # overall_plot_1.legend(shadow=True)
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot_2 = plt.subplot(3, 2, 3)
    plt.title("Y")
    plt.plot(complete_timestamps, prediction_yc -
             GT_yc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_yc-GT_yc,
             'c--', label='error odom - GT')
    # if(show_errors):
    #     overall_plot_2.legend(["error pred - GT\nmean: %.5f" % mean_ep_y +
    #                            " std: %.5f" % SD_ep_y, "error odom - GT\nmean: %.5f" % mean_eo_y+" std: %.5f" % SD_eo_y], shadow=True)
    # else:
    # overall_plot_2.legend(shadow=True)
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot_3 = plt.subplot(3, 2, 5)
    plt.title("Z")
    plt.plot(complete_timestamps, prediction_zc -
             GT_zc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_zc-GT_zc,
             'c--', label='error odom - GT')
    # if(show_errors):
    #     overall_plot_3.legend(["error pred - GT\nmean: %.5f" % mean_ep_z +
    #                            " std: %.5f" % SD_ep_z, "error odom - GT\nmean: %.5f" % mean_eo_z+" std: %.5f" % SD_eo_z], shadow=True)
    # else:
    # overall_plot_3.legend(shadow=True)
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot_4 = plt.subplot(3, 2, 2)
    plt.title("Roll")
    plt.plot(complete_timestamps, prediction_rollc -
             GT_rollc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_rollc -
             GT_rollc, 'c--', label='error odom - GT')
    # if(show_errors):
    #     overall_plot_4.legend(["error pred - GT\nmean: %.5f" % mean_ep_roll +
    #                            " std: %.5f" % SD_ep_roll, "error odom - GT\nmean: %.5f" % mean_eo_roll+" std: %.5f" % SD_eo_roll], shadow=True)
    # else:
    # overall_plot_4.legend(shadow=True)
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot_5 = plt.subplot(3, 2, 4)
    plt.title("Pitch")
    plt.plot(complete_timestamps, prediction_pitchc -
             GT_pitchc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_pitchc -
             GT_pitchc, 'c--', label='error odom - GT')
    # if(show_errors):
    #     overall_plot_5.legend(["error pred - GT\nmean: %.5f" % mean_ep_pitch +
    #                            " std: %.5f" % SD_ep_pitch, "error odom - GT\nmean: %.5f" % mean_eo_pitch+" std: %.5f" % SD_eo_pitch], shadow=True)
    # else:
    # overall_plot_5.legend(shadow=True)
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot_6 = plt.subplot(3, 2, 6)
    plt.title("Yaw")
    plt.plot(complete_timestamps, prediction_yawc -
             GT_yawc, 'r', label='error pred - GT')
    plt.plot(complete_timestamps, odom_yawc -
             GT_yawc, 'c--', label='error odom - GT')
    # if(show_errors):
    #     overall_plot_6.legend(["error pred - GT\nmean: %.5f" % mean_ep_yaw +
    #                            " std: %.5f" % SD_ep_yaw, "error odom - GT\nmean: %.5f" % mean_eo_yaw+" std: %.5f" % SD_eo_yaw], shadow=True)
    # else:
    # overall_plot_6.legend(shadow=True)
    plt.ylabel('Total yaw angle [°]')
    plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 7)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 8)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    figs = plt.figure()

    step_graph_1 = plt.subplot(3, 2, 1)
    plt.title("X")
    plt.plot(step_timestamps, prediction_xs -
             GT_xs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_xs-GT_xs, 'c--', label='error odom - GT')
    # if(show_errors):
    #     step_graph_1.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_x +
    #                         " std: %.5f" % step_SD_ep_x, "error odom - GT\nmean: %.5f" % step_mean_eo_x+" std: %.5f" % step_SD_eo_x], shadow=True)
    # else:
    #     step_graph_1.legend(shadow=True)
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph_2 = plt.subplot(3, 2, 3)
    plt.title("Y")
    plt.plot(step_timestamps, prediction_ys -
             GT_ys, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_ys-GT_ys, 'c--', label='error odom - GT')
    # if(show_errors):
    #     step_graph_2.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_y +
    #                         " std: %.5f" % step_SD_ep_y, "error odom - GT\nmean: %.5f" % step_mean_eo_y+" std: %.5f" % step_SD_eo_y], shadow=True)
    # else:
    #     step_graph_2.legend(shadow=True)
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph_3 = plt.subplot(3, 2, 5)
    plt.title("Z")
    plt.plot(step_timestamps, prediction_zs -
             GT_zs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_zs-GT_zs, 'c--', label='error odom - GT')
    # if(show_errors):
    #     step_graph_3.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_z +
    #                         " std: %.5f" % step_SD_ep_z, "error odom - GT\nmean: %.5f" % step_mean_eo_z+" std: %.5f" % step_SD_eo_z], shadow=True)
    # else:
    #     step_graph_3.legend(shadow=True)
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph_4 = plt.subplot(3, 2, 2)
    plt.title("Roll")
    plt.plot(step_timestamps, prediction_rolls -
             GT_rolls, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_rolls-GT_rolls,
             'c--', label='error odom - GT')
    # if(show_errors):
    #     step_graph_4.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_roll +
    #                         " std: %.5f" % step_SD_ep_roll, "error odom - GT\nmean: %.5f" % step_mean_eo_roll+" std: %.5f" % step_SD_eo_roll], shadow=True)
    # else:
    #     step_graph_4.legend(shadow=True)
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph_5 = plt.subplot(3, 2, 4)
    plt.title("Pitch")
    plt.plot(step_timestamps, prediction_pitchs -
             GT_pitchs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_pitchs -
             GT_pitchs, 'c--', label='error odom - GT')
    # if(show_errors):
    #     step_graph_5.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_pitch +
    #                         " std: %.5f" % step_SD_ep_pitch, "error odom - GT\nmean: %.5f" % step_mean_eo_pitch+" std: %.5f" % step_SD_eo_pitch], shadow=True)
    # else:
    # step_graph_5.legend(shadow=True)
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph_6 = plt.subplot(3, 2, 6)
    plt.title("Yaw")
    plt.plot(step_timestamps, prediction_yaws -
             GT_yaws, 'r', label='error pred - GT')
    plt.plot(step_timestamps, odom_yaws-GT_yaws,
             'c--', label='error odom - GT')
    # if(show_errors):
    #     step_graph_6.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_yaw +
    #                         " std: %.5f" % step_SD_ep_yaw, "error odom - GT\nmean: %.5f" % step_mean_eo_yaw+" std: %.5f" % step_SD_eo_yaw], shadow=True)
    # else:
    #     step_graph_6.legend(shadow=True)
    plt.ylabel('step yaw angle [°]')
    plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 7)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    # feature_plot = plt.subplot(4, 2, 8)
    # plt.title("Matches", fontsize=11)
    # plt.plot(feature_timestamps, feature_number,
    #          'm', label="# of Features per step")
    # # feature_plot.legend(shadow=True)
    # plt.ylabel('Feature Number')
    # plt.grid(True, 'both')

    figc.set_figheight(15)
    figs.set_figheight(15)
    figc.set_figwidth(15)
    figs.set_figwidth(15)
    plt.show()
    if(printBool):
        if(show_errors):
            if(extractor == "orb"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" +
                             length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
            elif(extractor == "brisk"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" +
                             length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:" +
                             brisk_pattern_scale+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length +
                             "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
            else:
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')

        else:
            if(extractor == "orb"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" +
                             length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
            elif(extractor == "brisk"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" +
                             length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:" +
                             brisk_pattern_scale+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_length:" + length +
                             "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
            else:
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')


# def plot_trajectory(printBool, extractor):
    # path_figure = plt.figure()
    # pathplot = plt.subplot(1, 1, 1)
    # plt.title("Path Plot")
    # plt.plot(prediction_xc, prediction_yc, 'b', label='x vs y Prediction')
    # plt.plot(GT_xc, GT_yc, 'g', label='x vs y GT')
    # plt.plot(odom_xc, odom_yc, 'y', label='x vs y Odom')
    # plt.ylabel('y')
    # pathplot.legend(shadow=True)
    # path_figure.set_figheight(15)
    # path_figure.set_figwidth(15)
    # plt.show()
    # if(printBool):
    #     if(extractor == "orb"):
    #         path_figure.savefig("pdfs/Traj" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
    #                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "_smooth:" + smoothing+"_length:" + length +
    #                             "_minTracked:"+klt_min_tracked + "detQuality:"+klt_quality_level + "blocksize:"+klt_block_size +
    #                             "minDist:"+klt_min_klt_distance+"ErrorThresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
    #                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')
    #     elif(extractor == "brisk"):
    #         path_figure.savefig("pdfs/Traj" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
    #                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "smooth:" + smoothing+"_length:" + length +
    #                             "_minTracked:"+klt_min_tracked + "detQuality:"+klt_quality_level + "blocksize:"+klt_block_size +
    #                             "minDist:"+klt_min_klt_distance+"ErrorThresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
    #                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')
    #     else:
    #         path_figure.savefig("pdfs/Traj" + "_Data:" + Data + "_Extr.:" + Extractor + "_Mask:" + MASK + "dupl_size:"+duplicate_size+"_min_dist:" +
    #                             min_distance + "_max_distance:" + max_match_distance + "_max_cos:" + max_cos + "smooth:" + smoothing+"_length:" + length +
    #                             "_minTracked:"+klt_min_tracked + "detQuality:"+klt_quality_level + "blocksize:"+klt_block_size +
    #                             "minDist:"+klt_min_klt_distance+"ErrorThresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
    #                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')


def plot_trajectory(printBool, extractor):
    path_figure = plt.figure()
    pathplot = plt.subplot(1, 1, 1)
    plt.title("My Method", fontsize=20)
    plt.plot(prediction_xc, prediction_yc, 'b', label='My Method')
    plt.plot(GT_xc, GT_yc, 'g', label='Lio Sam Ground Truth')
    # plt.plot(odom_xc, odom_yc, 'y', label='LOAM Scan2Scan')
    # plt.plot(odom1_xc, odom1_yc, 'r', label='LOAM Scan2Map')
    plt.ylabel('y[m]', fontsize=20)
    plt.xlabel('x[m]', fontsize=20)
    pathplot.legend(shadow=True)
    # plt.xlim(-250, 250)
    # plt.ylim(-100, 300)
    path_figure.set_figheight(20)
    path_figure.set_figwidth(10)
    plt.show()
    if(printBool):
        path_figure.savefig("pdfs/My_Method_Trajectory.pdf",
                            bbox_inches='tight')


if __name__ == "__main__":

    prediction_pose = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + directory + "/prediction_pose_"+file_name+".csv", nrows=file_length)
    prediction_xc = pd.DataFrame.to_numpy(prediction_pose["x"])
    prediction_yc = pd.DataFrame.to_numpy(prediction_pose["y"])
    prediction_zc = pd.DataFrame.to_numpy(prediction_pose["z"])
    prediction_rollc = pd.DataFrame.to_numpy(prediction_pose["roll"])
    prediction_pitchc = pd.DataFrame.to_numpy(prediction_pose["pitch"])
    prediction_yawc = pd.DataFrame.to_numpy(prediction_pose["yaw"])

    prediction_step = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + directory + "/prediction_steps_"+file_name+".csv", nrows=file_length)
    prediction_xs = pd.DataFrame.to_numpy(prediction_step["x"])
    prediction_ys = pd.DataFrame.to_numpy(prediction_step["y"])
    prediction_zs = pd.DataFrame.to_numpy(prediction_step["z"])
    prediction_rolls = pd.DataFrame.to_numpy(prediction_step["roll"])
    prediction_pitchs = pd.DataFrame.to_numpy(prediction_step["pitch"])
    prediction_yaws = pd.DataFrame.to_numpy(prediction_step["yaw"])

    GT_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_steps.csv", nrows=file_length
    )

    GT_xs = pd.DataFrame.to_numpy(GT_steps["x"])
    GT_ys = pd.DataFrame.to_numpy(GT_steps["y"])
    GT_zs = pd.DataFrame.to_numpy(GT_steps["z"])
    GT_rolls = pd.DataFrame.to_numpy(GT_steps["roll"])
    GT_pitchs = pd.DataFrame.to_numpy(GT_steps["pitch"])
    GT_yaws = pd.DataFrame.to_numpy(GT_steps["yaw"])

    GT_overall = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_pose.csv", nrows=file_length
    )

    GT_xc = pd.DataFrame.to_numpy(GT_overall["x"])
    GT_yc = pd.DataFrame.to_numpy(GT_overall["y"])
    GT_zc = pd.DataFrame.to_numpy(GT_overall["z"])
    GT_rollc = pd.DataFrame.to_numpy(GT_overall["roll"])
    GT_pitchc = pd.DataFrame.to_numpy(GT_overall["pitch"])
    GT_yawc = pd.DataFrame.to_numpy(GT_overall["yaw"])

    odom_file_complete = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/loam_pose_scan_2_scan.csv", nrows=file_length
    )
    odom_xc = pd.DataFrame.to_numpy(odom_file_complete["x"])
    odom_yc = pd.DataFrame.to_numpy(odom_file_complete["y"])
    odom_zc = pd.DataFrame.to_numpy(odom_file_complete["z"])
    odom_rollc = pd.DataFrame.to_numpy(odom_file_complete["roll"])
    odom_pitchc = pd.DataFrame.to_numpy(odom_file_complete["pitch"])
    odom_yawc = pd.DataFrame.to_numpy(odom_file_complete["yaw"])

    odom_file_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/loam_steps_scan_2_scan.csv", nrows=file_length
    )

    odom_xs = pd.DataFrame.to_numpy(odom_file_steps["x"])
    odom_ys = pd.DataFrame.to_numpy(odom_file_steps["y"])
    odom_zs = pd.DataFrame.to_numpy(odom_file_steps["z"])
    odom_rolls = pd.DataFrame.to_numpy(odom_file_steps["roll"])
    odom_pitchs = pd.DataFrame.to_numpy(odom_file_steps["pitch"])
    odom_yaws = pd.DataFrame.to_numpy(odom_file_steps["yaw"])

    odom_file1_complete = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/loam_pose_scan_2_map.csv", nrows=file_length
    )
    odom1_xc = pd.DataFrame.to_numpy(odom_file1_complete["x"])
    odom1_yc = pd.DataFrame.to_numpy(odom_file1_complete["y"])
    odom1_zc = pd.DataFrame.to_numpy(odom_file1_complete["z"])
    odom1_rollc = pd.DataFrame.to_numpy(odom_file1_complete["roll"])
    odom1_pitchc = pd.DataFrame.to_numpy(odom_file1_complete["pitch"])
    odom1_yawc = pd.DataFrame.to_numpy(odom_file1_complete["yaw"])

    odom_file1_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/loam_steps_scan_2_map.csv", nrows=file_length
    )

    odom1_xs = pd.DataFrame.to_numpy(odom_file1_steps["x"])
    odom1_ys = pd.DataFrame.to_numpy(odom_file1_steps["y"])
    odom1_zs = pd.DataFrame.to_numpy(odom_file1_steps["z"])
    odom1_rolls = pd.DataFrame.to_numpy(odom_file1_steps["roll"])
    odom1_pitchs = pd.DataFrame.to_numpy(odom_file1_steps["pitch"])
    odom1_yaws = pd.DataFrame.to_numpy(odom_file1_steps["yaw"])

    feature_number_file = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + directory + "/feature_number_"+file_name+".csv", nrows=file_length
    )

    feature_number = pd.DataFrame.to_numpy(
        feature_number_file["num_of_features"])

    # create timestamps

    complete_timestamps = pd.DataFrame.to_numpy(
        prediction_pose["time"])
    step_timestamps = pd.DataFrame.to_numpy(prediction_step["time"])
    odom_s_timestamps = pd.DataFrame.to_numpy(odom_file_steps["time"])
    odom_c_timestamps = pd.DataFrame.to_numpy(odom_file_complete["time"])

    feature_timestamps = pd.DataFrame.to_numpy(feature_number_file["time"])
    feature_timestamps = feature_timestamps - complete_timestamps[0]
    step_timestamps = step_timestamps - complete_timestamps[0]
    odom_s_timestamps = odom_s_timestamps - complete_timestamps[0]
    odom_c_timestamps = odom_c_timestamps - complete_timestamps[0]
    complete_timestamps = complete_timestamps - complete_timestamps[0]

    # plot_values(False, "orb")
    # plot_errors(False, False, "orb")
    plot_trajectory(False, "orb")

    # calculation of average error:
    diffx = prediction_xs-GT_xs
    diffy = prediction_ys-GT_ys
    diffz = prediction_zs-GT_zs
    diffroll = prediction_rolls-GT_rolls
    diffpitch = prediction_pitchs-GT_pitchs
    diffyaw = prediction_yaws-GT_yaws

    average_x = np.sum(np.abs(diffx))/(int(length)*10)
    average_y = np.sum(np.abs(diffy))/(int(length)*10)
    average_z = np.sum(np.abs(diffz))/(int(length)*10)
    average_roll = np.sum(np.abs(diffroll))/(int(length)*10)
    average_pitch = np.sum(np.abs(diffpitch))/(int(length)*10)
    average_yaw = np.sum(np.abs(diffyaw))/(int(length)*10)

    print(file_name)
    print("Average diff x: ", average_x)
    print("Average diff y: ", average_y)
    print("Average diff z: ", average_z)
    print("Average diff roll: ", average_roll)
    print("Average diff pitch: ", average_pitch)
    print("Average diff yaw: ", average_yaw)
