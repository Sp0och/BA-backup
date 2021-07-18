import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

file_name = "orb_0.3_3_2"
directory = "output"

Data = "Intensity"
Extractor = "orb"
duplicate_size = "3"
max_depth_distance = "0.3m"
min_distance = "0.1m"
smoothing = "2"
length = "11983"  # steps (10Hz), not seconds
file_length = 11983

# orb parameters
orb_max_features = "1000"
orb_accuracy = "31"
orb_scale_factor = "1.7"
orb_levels = "8"

# brisk parameters
brisk_threshold = "45"
brisk_octaves = "3"
brisk_pattern_scale = "0.5"

# klt parameters
klt_min_tracked = "70"
klt_quality_level = "0.05"
klt_block_size = "3"
klt_min_klt_distance = "3"

klt_epsilon = "0.005"
klt_criteria_reps = "10"
klt_opt_size = "17"
klt_num_pyramids = "2"


def plot_values(storeBool, showLegend, extractor):
    figc = plt.figure()
    overall_plot_1 = plt.subplot(4, 2, 1)
    plt.title("X")
    plt.plot(loam_p_timestamps, loam_xp, 'y', label='LOAM S2S')
    plt.plot(pose_timestamps, prediction_xc, 'b', label='My Method')
    plt.plot(pose_timestamps, GT_xc, 'g', label='Lio-Sam GT')
    if(showLegend):
        overall_plot_1.legend(shadow=True)
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot_2 = plt.subplot(4, 2, 3)
    plt.title("Y")
    plt.plot(loam_p_timestamps, loam_yp, 'y', label='LOAM S2S')
    plt.plot(pose_timestamps, prediction_yc, 'b', label='My Method')
    plt.plot(pose_timestamps, GT_yc, 'g', label='Lio-Sam GT')
    if(showLegend):
        overall_plot_2.legend(shadow=True)
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot_3 = plt.subplot(4, 2, 5)
    plt.title("Z")
    plt.plot(loam_p_timestamps, loam_zp, 'y', label='LOAM S2S')
    plt.plot(pose_timestamps, prediction_zc, 'b', label='My Method')
    plt.plot(pose_timestamps, GT_zc, 'g', label='Lio-Sam GT')
    if(showLegend):
        overall_plot_3.legend(shadow=True)
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot_4 = plt.subplot(4, 2, 2)
    plt.title("Roll")
    plt.plot(loam_p_timestamps, 57.2858*loam_rollp,
             'y', label='LOAM S2S')
    plt.plot(pose_timestamps, 57.2858 *
             prediction_rollc, 'b', label='My Method')
    plt.plot(pose_timestamps, 57.2858 *
             GT_rollc, 'g', label='Lio-Sam GT')
    if(showLegend):
        overall_plot_4.legend(shadow=True)
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot_5 = plt.subplot(4, 2, 4)
    plt.title("Pitch")
    plt.plot(loam_p_timestamps, 57.2858*loam_pitchp,
             'y', label='LOAM S2S')
    plt.plot(pose_timestamps, 57.2858 *
             prediction_pitchc, 'b', label='My Method')
    plt.plot(pose_timestamps, 57.2858 *
             GT_pitchc, 'g', label='Lio-Sam GT')
    if(showLegend):
        overall_plot_5.legend(shadow=True)
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot_6 = plt.subplot(4, 2, 6)
    plt.title("Yaw")
    plt.plot(loam_p_timestamps, 57.2858*loam_yawp,
             'y', label='LOAM S2S')
    plt.plot(pose_timestamps, 57.2858 *
             prediction_yawc, 'b', label='My Method')
    plt.plot(pose_timestamps, 57.2858*GT_yawc, 'g', label='Lio-Sam GT')
    if(showLegend):
        overall_plot_6.legend(shadow=True)
    plt.ylabel('Total yaw angle [°]')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 7)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 8)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    figs = plt.figure()

    step_graph_1 = plt.subplot(4, 2, 1)
    plt.title("X")
    plt.plot(loam_s_timestamps, loam_xs, 'y', label='LOAM S2S')
    plt.plot(step_timestamps, prediction_xs, 'b--', label='My Method')
    plt.plot(step_timestamps, GT_xs, 'g--', label='Lio-Sam GT')
    if(showLegend):
        step_graph_1.legend(shadow=True)
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph_2 = plt.subplot(4, 2, 3)
    plt.title("Y")
    plt.plot(loam_s_timestamps, loam_ys, 'y', label='LOAM S2S')
    plt.plot(step_timestamps, prediction_ys, 'b--', label='My Method')
    plt.plot(step_timestamps, GT_ys, 'g--', label='Lio-Sam GT')
    if(showLegend):
        step_graph_2.legend(shadow=True)
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph_3 = plt.subplot(4, 2, 5)
    plt.title("Z")
    plt.plot(loam_s_timestamps, loam_zs, 'y', label='LOAM S2S')
    plt.plot(step_timestamps, prediction_zs, 'b--', label='My Method')
    plt.plot(step_timestamps, GT_zs, 'g--', label='Lio-Sam GT')
    if(showLegend):
        step_graph_3.legend(shadow=True)
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph_4 = plt.subplot(4, 2, 2)
    plt.title("Roll")
    plt.plot(step_timestamps, 57.2858 *
             GT_rolls, 'g--', label='Lio-Sam GT')
    plt.plot(step_timestamps, 57.2858 *
             prediction_rolls, 'b--', label='My Method')
    plt.plot(loam_s_timestamps, 57.2858*loam_rolls,
             'y', label='LOAM S2S')
    if(showLegend):
        step_graph_4.legend(shadow=True)
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph_5 = plt.subplot(4, 2, 4)
    plt.title("Pitch")
    plt.plot(step_timestamps, 57.2858 *
             GT_pitchs, 'g--', label='Lio-Sam GT')
    plt.plot(step_timestamps, 57.2858 *
             prediction_pitchs, 'b--', label='My Method')
    plt.plot(loam_s_timestamps, 57.2858*loam_pitchs,
             'y', label='LOAM S2S')
    if(showLegend):
        step_graph_5.legend(shadow=True)
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph_6 = plt.subplot(4, 2, 6)
    plt.title("Yaw")
    plt.plot(step_timestamps, 57.2858*GT_yaws, 'g--', label='Lio-Sam GT')
    plt.plot(step_timestamps, 57.2858 *
             prediction_yaws, 'b--', label='My Method')
    plt.plot(loam_s_timestamps, 57.2858*loam_yaws,
             'y', label='LOAM S2S')
    if(showLegend):
        step_graph_6.legend(shadow=True)
    plt.ylabel('step yaw angle [°]')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 7)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 8)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    figc.set_figheight(15)
    figs.set_figheight(15)
    figc.set_figwidth(15)
    figs.set_figwidth(15)
    plt.show()
    if(storeBool):
        if(extractor == "orb"):
            figc.savefig("pdfs/Pose" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_depth_distance + "_smooth:" + smoothing+"_length:" +
                         length + "max_orb_feat:" + orb_max_features +
                         "orb_acc:" + orb_accuracy + "orb_Sc_Fa:"
                         + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_depth_distance + "_smooth:" + smoothing+"_length:" +
                         length + "max_orb_feat:" + orb_max_features+"orb_acc:" + orb_accuracy + "orb_Sc_Fa:" +
                         orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
        elif(extractor == "brisk"):
            figc.savefig("pdfs/Pose" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_P_S:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_P_S:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
        else:
            figc.savefig("pdfs/Pose" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_det_Quality:"+klt_quality_level + "_block_size:"+klt_block_size +
                         "_min_Det_Dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
            figs.savefig("pdfs/Steps" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                         min_distance + "_max_dist:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                         "_minTracked:"+klt_min_tracked + "_det_Quality:"+klt_quality_level + "_block_size:"+klt_block_size +
                         "_min_Det_Dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                         "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')


def plot_errors(storeBool, showLegend, show_errors, extractor):

    error_prediction_x = np.abs(prediction_xc - GT_xc)
    error_prediction_y = np.abs(prediction_yc - GT_yc)
    error_prediction_z = np.abs(prediction_zc - GT_zc)
    error_prediction_roll = np.abs(57.2858*(prediction_rollc - GT_rollc))
    error_prediction_pitch = np.abs(57.2858*(prediction_pitchc - GT_pitchc))
    error_prediction_yaw = np.abs(57.2858*(prediction_yawc - GT_yawc))

    error_odom_x = np.abs(loam_xp - GT_xc)
    error_odom_y = np.abs(loam_yp - GT_yc)
    error_odom_z = np.abs(loam_zp - GT_zc)
    error_odom_roll = np.abs(57.2858*(loam_rollp - GT_rollc))
    error_odom_pitch = np.abs(57.2858*(loam_pitchp - GT_pitchc))
    error_odom_yaw = np.abs(57.2858*(loam_yawp - GT_yawc))

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

    step_error_odom_x = np.abs(loam_xs - GT_xs)
    step_error_odom_y = np.abs(loam_ys - GT_ys)
    step_error_odom_z = np.abs(loam_zs - GT_zs)
    step_error_odom_roll = np.abs(57.2858*(loam_rolls - GT_rolls))
    step_error_odom_pitch = np.abs(57.2858*(loam_pitchs - GT_pitchs))
    step_error_odom_yaw = np.abs(57.2858*(loam_yaws - GT_yaws))

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
    plt.title("X")
    plt.plot(pose_timestamps, prediction_xc -
             GT_xc, 'r', label='error My Method vs Lio-Sam GT')
    plt.plot(pose_timestamps, loam_xp-GT_xc,
             'c--', label='error LOAM S2S vs  - Lio-Sam GT')
    if(showLegend):
        if(show_errors):
            overall_plot_1.legend(
                ["error pred - GT\nmean: %.5f" % mean_ep_x+" std: %.5f" % SD_ep_x, "error odom - GT\nmean: %.5f" % mean_eo_x+" std: %.5f" % SD_eo_x], shadow=True)
        else:
            overall_plot_1.legend(shadow=True)
    plt.ylabel('Total Transl x [m]')
    plt.grid(True, 'both')

    overall_plot_2 = plt.subplot(4, 2, 3)
    plt.title("Y")
    plt.plot(pose_timestamps, prediction_yc -
             GT_yc, 'r', label='error pred - GT')
    plt.plot(pose_timestamps, loam_yp-GT_yc,
             'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            overall_plot_2.legend(["error pred - GT\nmean: %.5f" % mean_ep_y +
                                   " std: %.5f" % SD_ep_y, "error odom - GT\nmean: %.5f" % mean_eo_y+" std: %.5f" % SD_eo_y], shadow=True)
        else:
            overall_plot_2.legend(shadow=True)
    plt.ylabel('Total Transl y [m]')
    plt.grid(True, 'both')

    overall_plot_3 = plt.subplot(4, 2, 5)
    plt.title("Z")
    plt.plot(pose_timestamps, prediction_zc -
             GT_zc, 'r', label='error pred - GT')
    plt.plot(pose_timestamps, loam_zp-GT_zc,
             'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            overall_plot_3.legend(["error pred - GT\nmean: %.5f" % mean_ep_z +
                                   " std: %.5f" % SD_ep_z, "error odom - GT\nmean: %.5f" % mean_eo_z+" std: %.5f" % SD_eo_z], shadow=True)
        else:
            overall_plot_3.legend(shadow=True)
    plt.ylabel('Total Transl z [m]')
    plt.grid(True, 'both')

    overall_plot_4 = plt.subplot(4, 2, 2)
    plt.title("Roll")
    plt.plot(pose_timestamps, prediction_rollc -
             GT_rollc, 'r', label='error pred - GT')
    plt.plot(pose_timestamps, loam_rollp -
             GT_rollc, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            overall_plot_4.legend(["error pred - GT\nmean: %.5f" % mean_ep_roll +
                                   " std: %.5f" % SD_ep_roll, "error odom - GT\nmean: %.5f" % mean_eo_roll+" std: %.5f" % SD_eo_roll], shadow=True)
        else:
            overall_plot_4.legend(shadow=True)
    plt.ylabel('Total row angle [°]')
    plt.grid(True, 'both')

    overall_plot_5 = plt.subplot(4, 2, 4)
    plt.title("Pitch")
    plt.plot(pose_timestamps, prediction_pitchc -
             GT_pitchc, 'r', label='error pred - GT')
    plt.plot(pose_timestamps, loam_pitchp -
             GT_pitchc, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            overall_plot_5.legend(["error pred - GT\nmean: %.5f" % mean_ep_pitch +
                                   " std: %.5f" % SD_ep_pitch, "error odom - GT\nmean: %.5f" % mean_eo_pitch+" std: %.5f" % SD_eo_pitch], shadow=True)
        else:
            overall_plot_5.legend(shadow=True)
    plt.ylabel('Total pitch angle [°]')
    plt.grid(True, 'both')

    overall_plot_6 = plt.subplot(4, 2, 6)
    plt.title("Yaw")
    plt.plot(pose_timestamps, prediction_yawc -
             GT_yawc, 'r', label='error pred - GT')
    plt.plot(pose_timestamps, loam_yawp -
             GT_yawc, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            overall_plot_6.legend(["error pred - GT\nmean: %.5f" % mean_ep_yaw +
                                   " std: %.5f" % SD_ep_yaw, "error odom - GT\nmean: %.5f" % mean_eo_yaw+" std: %.5f" % SD_eo_yaw], shadow=True)
        else:
            overall_plot_6.legend(shadow=True)
    plt.ylabel('Total yaw angle [°]')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 7)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 8)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    figs = plt.figure()

    step_graph_1 = plt.subplot(4, 2, 1)
    plt.title("X")
    plt.plot(step_timestamps, prediction_xs -
             GT_xs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, loam_xs-GT_xs, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            step_graph_1.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_x +
                                " std: %.5f" % step_SD_ep_x, "error odom - GT\nmean: %.5f" % step_mean_eo_x+" std: %.5f" % step_SD_eo_x], shadow=True)
        else:
            step_graph_1.legend(shadow=True)
    plt.ylabel('step Transl x [m]')
    plt.grid(True, 'both')

    step_graph_2 = plt.subplot(4, 2, 3)
    plt.title("Y")
    plt.plot(step_timestamps, prediction_ys -
             GT_ys, 'r', label='error pred - GT')
    plt.plot(step_timestamps, loam_ys-GT_ys, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            step_graph_2.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_y +
                                " std: %.5f" % step_SD_ep_y, "error odom - GT\nmean: %.5f" % step_mean_eo_y+" std: %.5f" % step_SD_eo_y], shadow=True)
        else:
            step_graph_2.legend(shadow=True)
    plt.ylabel('step Transl y [m]')
    plt.grid(True, 'both')

    step_graph_3 = plt.subplot(4, 2, 5)
    plt.title("Z")
    plt.plot(step_timestamps, prediction_zs -
             GT_zs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, loam_zs-GT_zs, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            step_graph_3.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_z +
                                " std: %.5f" % step_SD_ep_z, "error odom - GT\nmean: %.5f" % step_mean_eo_z+" std: %.5f" % step_SD_eo_z], shadow=True)
        else:
            step_graph_3.legend(shadow=True)
    plt.ylabel('step Transl z [m]')
    plt.grid(True, 'both')

    step_graph_4 = plt.subplot(4, 2, 2)
    plt.title("Roll")
    plt.plot(step_timestamps, prediction_rolls -
             GT_rolls, 'r', label='error pred - GT')
    plt.plot(step_timestamps, loam_rolls-GT_rolls,
             'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            step_graph_4.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_roll +
                                " std: %.5f" % step_SD_ep_roll, "error odom - GT\nmean: %.5f" % step_mean_eo_roll+" std: %.5f" % step_SD_eo_roll], shadow=True)
        else:
            step_graph_4.legend(shadow=True)
    plt.ylabel('step row angle [°]')
    plt.grid(True, 'both')

    step_graph_5 = plt.subplot(4, 2, 4)
    plt.title("Pitch")
    plt.plot(step_timestamps, prediction_pitchs -
             GT_pitchs, 'r', label='error pred - GT')
    plt.plot(step_timestamps, loam_pitchs -
             GT_pitchs, 'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            step_graph_5.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_pitch +
                                " std: %.5f" % step_SD_ep_pitch, "error odom - GT\nmean: %.5f" % step_mean_eo_pitch+" std: %.5f" % step_SD_eo_pitch], shadow=True)
        else:
            step_graph_5.legend(shadow=True)
    plt.ylabel('step pitch angle [°]')
    plt.grid(True, 'both')

    step_graph_6 = plt.subplot(4, 2, 6)
    plt.title("Yaw")
    plt.plot(step_timestamps, prediction_yaws -
             GT_yaws, 'r', label='error pred - GT')
    plt.plot(step_timestamps, loam_yaws-GT_yaws,
             'c--', label='error odom - GT')
    if(showLegend):
        if(show_errors):
            step_graph_6.legend(["error pred - GT\nmean: %.5f" % step_mean_ep_yaw +
                                " std: %.5f" % step_SD_ep_yaw, "error odom - GT\nmean: %.5f" % step_mean_eo_yaw+" std: %.5f" % step_SD_eo_yaw], shadow=True)
        else:
            step_graph_6.legend(shadow=True)
    plt.ylabel('step yaw angle [°]')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 7)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    feature_plot = plt.subplot(4, 2, 8)
    plt.title("Matches", fontsize=11)
    plt.plot(feature_timestamps, feature_number,
             'm', label="# of Features per step")
    if(showLegend):
        feature_plot.legend(shadow=True)
    plt.ylabel('Feature Number')
    plt.grid(True, 'both')

    figc.set_figheight(15)
    figs.set_figheight(15)
    figc.set_figwidth(15)
    figs.set_figwidth(15)
    plt.show()
    if(storeBool):
        if(show_errors):
            if(extractor == "orb"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" + length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" +
                             length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
            elif(extractor == "brisk"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" +
                             length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:" +
                             brisk_pattern_scale+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" + length +
                             "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
            else:
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')

        else:
            if(extractor == "orb"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" + length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" +
                             length + "max_orb_feat:" + orb_max_features +
                             "orb_accuracy:" + orb_accuracy + "orb_Sc_Fa:"
                             + orb_scale_factor + "orb_levels:" + orb_levels+".pdf", bbox_inches='tight')
            elif(extractor == "brisk"):
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" +
                             length + "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:" +
                             brisk_pattern_scale+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_length:" + length +
                             "brisk_thresh:"+brisk_threshold + "octaves:" + brisk_octaves + "brisk_Pa_Sca:"+brisk_pattern_scale+".pdf", bbox_inches='tight')
            else:
                figc.savefig("pdfs/Path_error" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')
                figs.savefig("pdfs/Step_error" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                             min_distance + "_max_distance:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                             "_minTracked:"+klt_min_tracked + "_detQual:"+klt_quality_level + "_blocksize:"+klt_block_size +
                             "_min_det_dist:"+klt_min_klt_distance+"_Error_thresh:" + klt_epsilon+"_reps:"+klt_criteria_reps +
                             "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids+".pdf", bbox_inches='tight')


def plot_trajectory(storeBool, extractor):
    path_figure = plt.figure()
    pathplot = plt.subplot(1, 1, 1)
    plt.title("Path Plot")
    plt.plot(prediction_xc, prediction_yc, 'b', label='x vs y Prediction')
    plt.plot(GT_xc, GT_yc, 'g', label='x vs y GT')
    plt.plot(loam_xp, loam_yp, 'y', label='x vs y Odom')
    plt.ylabel('y')
    pathplot.legend(shadow=True)
    path_figure.set_figheight(15)
    path_figure.set_figwidth(15)
    plt.show()
    if(storeBool):
        if(extractor == "orb"):
            path_figure.savefig("pdfs/Traj" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                                min_distance + "_max_distance:" + max_depth_distance + "_smooth:" + smoothing+"_length:" + length +
                                "_minTracked:"+klt_min_tracked + "detQuality:"+klt_quality_level + "blocksize:"+klt_block_size +
                                "minDist:"+klt_min_klt_distance+"ErrorThresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                                "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')
        elif(extractor == "brisk"):
            path_figure.savefig("pdfs/Traj" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                                min_distance + "_max_distance:" + max_depth_distance + "smooth:" + smoothing+"_length:" + length +
                                "_minTracked:"+klt_min_tracked + "detQuality:"+klt_quality_level + "blocksize:"+klt_block_size +
                                "minDist:"+klt_min_klt_distance+"ErrorThresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                                "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')
        else:
            path_figure.savefig("pdfs/Traj" + "_Data:" + Data + "_Extr.:" + Extractor + "dupl_size:"+duplicate_size+"_min_dist:" +
                                min_distance + "_max_distance:" + max_depth_distance + "smooth:" + smoothing+"_length:" + length +
                                "_minTracked:"+klt_min_tracked + "detQuality:"+klt_quality_level + "blocksize:"+klt_block_size +
                                "minDist:"+klt_min_klt_distance+"ErrorThresh:" + klt_epsilon+"_matchingReps:"+klt_criteria_reps +
                                "_sizePerPyr:"+klt_opt_size+"_PyrLevels:"+klt_num_pyramids + ".pdf", bbox_inches='tight')


if __name__ == "__main__":

    prediction_pose = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/" + directory + "/prediction_pose_"+file_name+".csv", nrows=file_length)
    prediction_xc = pd.DataFrame.to_numpy(prediction_pose["x"])
    prediction_yc = pd.DataFrame.to_numpy(prediction_pose["y"])
    prediction_zc = pd.DataFrame.to_numpy(prediction_pose["z"])
    prediction_rollc = pd.DataFrame.to_numpy(prediction_pose["roll"])
    prediction_pitchc = pd.DataFrame.to_numpy(prediction_pose["pitch"])
    prediction_yawc = pd.DataFrame.to_numpy(prediction_pose["yaw"])

    prediction_step = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/" + directory + "/prediction_steps_"+file_name+".csv", nrows=file_length)
    prediction_xs = pd.DataFrame.to_numpy(prediction_step["x"])
    prediction_ys = pd.DataFrame.to_numpy(prediction_step["y"])
    prediction_zs = pd.DataFrame.to_numpy(prediction_step["z"])
    prediction_rolls = pd.DataFrame.to_numpy(prediction_step["roll"])
    prediction_pitchs = pd.DataFrame.to_numpy(prediction_step["pitch"])
    prediction_yaws = pd.DataFrame.to_numpy(prediction_step["yaw"])

    GT_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/output/GT_steps.csv", nrows=file_length
    )

    GT_xs = pd.DataFrame.to_numpy(GT_steps["x"])
    GT_ys = pd.DataFrame.to_numpy(GT_steps["y"])
    GT_zs = pd.DataFrame.to_numpy(GT_steps["z"])
    GT_rolls = pd.DataFrame.to_numpy(GT_steps["roll"])
    GT_pitchs = pd.DataFrame.to_numpy(GT_steps["pitch"])
    GT_yaws = pd.DataFrame.to_numpy(GT_steps["yaw"])

    GT_pose = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/output/GT_pose.csv", nrows=file_length
    )

    GT_xc = pd.DataFrame.to_numpy(GT_pose["x"])
    GT_yc = pd.DataFrame.to_numpy(GT_pose["y"])
    GT_zc = pd.DataFrame.to_numpy(GT_pose["z"])
    GT_rollc = pd.DataFrame.to_numpy(GT_pose["roll"])
    GT_pitchc = pd.DataFrame.to_numpy(GT_pose["pitch"])
    GT_yawc = pd.DataFrame.to_numpy(GT_pose["yaw"])

    loam_pose_file = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/output/loam_pose_scan_2_scan.csv", nrows=file_length
    )
    loam_xp = pd.DataFrame.to_numpy(loam_pose_file["x"])
    loam_yp = pd.DataFrame.to_numpy(loam_pose_file["y"])
    loam_zp = pd.DataFrame.to_numpy(loam_pose_file["z"])
    loam_rollp = pd.DataFrame.to_numpy(loam_pose_file["roll"])
    loam_pitchp = pd.DataFrame.to_numpy(loam_pose_file["pitch"])
    loam_yawp = pd.DataFrame.to_numpy(loam_pose_file["yaw"])

    loam_file_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/output/loam_steps_scan_2_scan.csv", nrows=file_length
    )

    loam_xs = pd.DataFrame.to_numpy(loam_file_steps["x"])
    loam_ys = pd.DataFrame.to_numpy(loam_file_steps["y"])
    loam_zs = pd.DataFrame.to_numpy(loam_file_steps["z"])
    loam_rolls = pd.DataFrame.to_numpy(loam_file_steps["roll"])
    loam_pitchs = pd.DataFrame.to_numpy(loam_file_steps["pitch"])
    loam_yaws = pd.DataFrame.to_numpy(loam_file_steps["yaw"])

    loam_map_file = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/output/loam_pose_scan_2_map.csv", nrows=file_length
    )
    loam_map_xp = pd.DataFrame.to_numpy(loam_map_file["x"])
    loam_map_yp = pd.DataFrame.to_numpy(loam_map_file["y"])
    loam_map_zp = pd.DataFrame.to_numpy(loam_map_file["z"])
    loam_map_rollp = pd.DataFrame.to_numpy(loam_map_file["roll"])
    loam_map_pitchp = pd.DataFrame.to_numpy(loam_map_file["pitch"])
    loam_map_yawp = pd.DataFrame.to_numpy(loam_map_file["yaw"])

    loam_map_steps = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/output/loam_steps_scan_2_map.csv", nrows=file_length
    )

    loam_map_xs = pd.DataFrame.to_numpy(loam_map_steps["x"])
    loam_map_ys = pd.DataFrame.to_numpy(loam_map_steps["y"])
    loam_map_zs = pd.DataFrame.to_numpy(loam_map_steps["z"])
    loam_map_rolls = pd.DataFrame.to_numpy(loam_map_steps["roll"])
    loam_map_pitchs = pd.DataFrame.to_numpy(loam_map_steps["pitch"])
    loam_map_yaws = pd.DataFrame.to_numpy(loam_map_steps["yaw"])

    feature_number_file = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/image_and_descriptor/" + directory + "/feature_number_"+file_name+".csv", nrows=file_length
    )

    feature_number = pd.DataFrame.to_numpy(
        feature_number_file["num_of_features"])

    # timestamps

    pose_timestamps = pd.DataFrame.to_numpy(
        prediction_pose["time"])
    step_timestamps = pd.DataFrame.to_numpy(prediction_step["time"])
    loam_s_timestamps = pd.DataFrame.to_numpy(loam_file_steps["time"])
    loam_p_timestamps = pd.DataFrame.to_numpy(loam_pose_file["time"])

    feature_timestamps = pd.DataFrame.to_numpy(feature_number_file["time"])
    feature_timestamps = feature_timestamps - pose_timestamps[0]
    step_timestamps = step_timestamps - pose_timestamps[0]
    loam_s_timestamps = loam_s_timestamps - pose_timestamps[0]
    loam_p_timestamps = loam_p_timestamps - pose_timestamps[0]
    pose_timestamps = pose_timestamps - pose_timestamps[0]

    plot_values(False, True,  Extractor)
    plot_errors(False, True, False, Extractor)
    plot_trajectory(False, Extractor)

    # calculation of average error:
    diffx = prediction_xs-GT_xs
    diffy = prediction_ys-GT_ys
    diffz = prediction_zs-GT_zs
    diffroll = prediction_rolls-GT_rolls
    diffpitch = prediction_pitchs-GT_pitchs
    diffyaw = prediction_yaws-GT_yaws

    # calculate loam error

    # diffx = loam_xs-GT_xs
    # diffy = loam_ys-GT_ys
    # diffz = loam_zs-GT_zs
    # diffroll = loam_rolls-GT_rolls
    # diffpitch = loam_pitchs-GT_pitchs
    # diffyaw = loam_yaws-GT_yaws

    # average_x = np.sum(np.abs(diffx))/file_length
    # average_y = np.sum(np.abs(diffy))/file_length
    # average_z = np.sum(np.abs(diffz))/file_length
    # average_roll = np.sum(np.abs(diffroll))/file_length
    # average_pitch = np.sum(np.abs(diffpitch))/file_length
    # average_yaw = np.sum(np.abs(diffyaw))/file_length

    # calculate this methods error

    mean_x = np.mean(diffx)
    mean_y = np.mean(diffy)
    mean_z = np.mean(diffz)
    mean_roll = np.mean(diffroll)
    mean_pitch = np.mean(diffpitch)
    mean_yaw = np.mean(diffyaw)

    SD_x = np.sqrt(np.sum(diffx*diffx)/file_length)
    SD_y = np.sqrt(np.sum(diffy*diffy)/file_length)
    SD_z = np.sqrt(np.sum(diffz*diffz)/file_length)
    SD_roll = np.sqrt(np.sum(diffroll*diffroll)/file_length)
    SD_pitch = np.sqrt(np.sum(diffpitch*diffpitch)/file_length)
    SD_yaw = np.sqrt(np.sum(diffyaw*diffyaw)/file_length)

    print(file_name)
    print("Mean Error x: ", mean_x)
    print("Mean Error y: ", mean_y)
    print("Mean Error z: ", mean_z)
    print("Mean Error roll: ", mean_roll)
    print("Mean Error pitch: ", mean_pitch)
    print("Mean Error yaw: ", mean_yaw)

    print("Error SD x: ", SD_x)
    print("Error SD y: ", SD_y)
    print("Error SD z: ", SD_z)
    print("Error SD roll: ", SD_roll)
    print("Error SD pitch: ", SD_pitch)
    print("Error SD yaw: ", SD_yaw)
