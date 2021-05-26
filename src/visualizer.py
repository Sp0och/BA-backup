import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    trans_x = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/translation_x.csv", delimiter='\n', max_rows=1000)
    trans_y = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/translation_y.csv", delimiter='\n', max_rows=1000)
    trans_z = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/translation_z.csv", delimiter='\n', max_rows=1000)
    rot_roll = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/rotation_roll.csv", delimiter='\n', max_rows=1000)
    rot_pitch = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/rotation_pitch.csv", delimiter='\n', max_rows=1000)
    rot_yaw = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/rotation_yaw.csv", delimiter='\n', max_rows=1000)
    s_trans_x = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_x.csv", delimiter='\n', max_rows=1000)
    s_trans_y = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_y.csv", delimiter='\n', max_rows=1000)
    s_trans_z = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_translation_z.csv", delimiter='\n', max_rows=1000)
    s_rot_roll = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_roll.csv", delimiter='\n', max_rows=1000)
    s_rot_pitch = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_pitch.csv", delimiter='\n', max_rows=1000)
    s_rot_yaw = np.loadtxt(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/s_rotation_yaw.csv", delimiter='\n', max_rows=1000)
    # set linear x axis
    x = np.linspace(0, 1000, 1000)
    # create the plots

    sub = plt.subplot(3, 2, 1)
    plt.title("Translation in x")
    plt.plot(x, trans_x, 'r')
    plt.plot(x, s_trans_x, 'b')
    # plt.xlabel('time')
    plt.ylabel('overall translation in x')

    sub = plt.subplot(3, 2, 3)
    plt.title("Translation in y")
    plt.plot(x, trans_y, 'r')
    plt.plot(x, s_trans_y, 'b')
    # plt.xlabel('time')
    plt.ylabel('overall translation in y')

    sub = plt.subplot(3, 2, 5)
    plt.title("Translation in z")
    plt.plot(x, trans_z, 'r')
    plt.plot(x, s_trans_z, 'b')
    # plt.xlabel('time')
    plt.ylabel('overall translation in z')

    sub = plt.subplot(3, 2, 2)
    plt.title("rotation roll")
    plt.plot(x, rot_roll, 'r')
    plt.plot(x, s_rot_roll, 'b')
    # plt.xlabel('time')
    plt.ylabel('overall row angle')
    plt.ylim([-5, 5])

    sub = plt.subplot(3, 2, 4)
    plt.title("Rotation pitch")
    plt.plot(x, rot_pitch, 'r')
    plt.plot(x, s_rot_pitch, 'b')
    # plt.xlabel('time')
    plt.ylabel('overall pitch angle')
    plt.ylim([-5, 5])

    sub = plt.subplot(3, 2, 6)
    plt.title("Rotation yaw")
    plt.plot(x, rot_yaw, 'r')
    plt.plot(x, s_rot_yaw, 'b')
    # plt.xlabel('time')
    plt.ylabel('overall yaw angle')
    plt.ylim([-5, 5])

    plt.show()
