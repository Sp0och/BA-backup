Languages:

c++
Python

Requirements:

ros
packages



General architecture:

    -The main node is in the main_node.cpp file
    -h files with the declarations are in the include folder while the definitions are in src
    -ImageHandler creates the projections from the rosbag data
    -ORB and BRISK are split into extractor and matching file (ORB / ORB_Framehandler)
    - notice that everything that needs consideration of two pointclouds or images is in the framehandler files while anything feature related is in the files named after the descriptors
    -most helping functions are defined in the helper.cpp file
    - finally KLT is implemented directly and completely in the KLT.cpp file.
    - variables beginning with M_ are member variables.


Filtering:
    - Points that get marked in the imagehandler as being too close to the sensor get filtered out in the get_3D_data function in the extractor classes.
    - Other filtering is applied visibly

Plotting:
"visualizer.py"
3 types of plotting:
    - estimated numbers compard to GT and LOAM (pose and steps)
    - errors regarding GT (pose and steps)
    - Trajectory
Filename as well as parameters can be set for notation at the very top of the file.