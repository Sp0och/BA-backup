import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    trans_x = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/translation_x.csv", delimiter=',')
    trans_y = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/translation_y.csv", delimiter=',')
    trans_z = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/translation_z.csv", delimiter=',')
    rot_x = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/rotation_yaw.csv", delimiter=',')
    rot_y = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/rotation_pitch.csv", delimiter=',')
    rot_z = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/rotation_roll.csv", delimiter=',')
    s_trans_x = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_x.csv", delimiter='\n')
    s_trans_y = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_y.csv", delimiter='\n')
    s_trans_z = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_z.csv", delimiter='\n')
    s_rot_x = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_yaw.csv", delimiter='\n')
    s_rot_y = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_pitch.csv", delimiter='\n')
    s_rot_z = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_roll.csv", delimiter='\n')
    # set linear x axis
    x = np.linspace(0, 1000, 1000)
    # create the plots

    sub = plt.subplot(3, 2, 1)
    plt.title("Translation in x")
    plt.plot(x, trans_x, 'r')
    plt.plot(x, s_trans_x, 'b')
    # plt.xlabel('time')
    plt.ylabel('translation in x')

    sub = plt.subplot(3, 2, 3)
    plt.title("Translation in y")
    plt.plot(x, trans_y, 'r')
    plt.plot(x, s_trans_y, 'b')
    plt.plot(x)
    # plt.xlabel('time')
    plt.ylabel('translation in y')

    sub = plt.subplot(3, 2, 5)
    plt.title("Translation in z")
    plt.plot(x, trans_z, 'r')
    plt.plot(x, s_trans_z, 'b')
    plt.plot(x)
    # plt.xlabel('time')
    plt.ylabel('translation in z')

    sub = plt.subplot(3, 2, 2)
    plt.title("rotation yaw")
    plt.plot(x, rot_x, 'r')
    plt.plot(x, s_rot_x, 'b')
    plt.plot(x)
    # plt.xlabel('time')
    plt.ylabel('yaw angle')
    plt.ylim([-5, 5])

    sub = plt.subplot(3, 2, 4)
    plt.title("Rotation pitch")
    plt.plot(x, rot_y, 'r')
    plt.plot(x, s_rot_y, 'b')
    plt.plot(x)
    # plt.xlabel('time')
    plt.ylabel('pitch angle')
    plt.ylim([-5, 5])

    sub = plt.subplot(3, 2, 6)
    plt.title("Rotation roll")
    plt.plot(x, rot_z, 'r')
    plt.plot(x, s_rot_z, 'b')
    plt.plot(x)
    # plt.xlabel('time')
    plt.ylabel('roll angle')
    plt.ylim([-5, 5])

    plt.show()
