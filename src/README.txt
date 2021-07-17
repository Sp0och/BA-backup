Languages:

c++
Python

Requirements:

ros
packages



General architecture:

    -libraries are included in the setup.h file
    -The main node is in the main_node.cpp file
    -h files with the declarations are in the include folder while the definitions are in src
    -ImageHandler creates the projections from the rosbag data
    -ORB and BRISK are split into extractor and matching file (ORB / ORB_Framehandler)
    - notice that everything that needs consideration of two pointclouds or images is in the *_Framehandler files while anything feature related is in the files named after the descriptors
    -most general functions are defined in the helper.cpp file
    - finally KLT is implemented directly and completely in the KLT.cpp file.
    - variables beginning with M_ are member variables.


Filtering:
    - Points that get marked in the imagehandler as being too close to the sensor get filtered out in the get_3D_data function in the extractor classes.
    - All other filtering instances are applied visibly and upfront

Plotting:
"visualizer.py"
3 types of plotting:
    - estimated numbers compard to GT and LOAM (pose and steps)
    - errors regarding GT (pose and steps)
    - Trajectory
Filename as well as parameters can be set for notation at the very top of the file.
The plots can be stored when passing the True parameter to the first respective function input - this can be done with legends (second argument) and for the error plots it can be chosen whether the mean and std of the errors should be displayed in the legends as well (consumes a lot of space)