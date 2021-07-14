Requirements:

ros
packages



Architecture:

-The main node is in the main_node.cpp file
-h files with the declarations are in the include folder while the definitions are in the source
-ImageHandler creates the projections from the rosbag data
-ORB and BRISK are split into extractor and matching file (ORB / ORB_Framehandler)
- notice that everything that needs consideration of two pointclouds or images is in the framehandler files while anything feature related is in the files named after the descriptors
-most helping functions are defined in the helper.cpp file
- finally KLT is implemented directly in the KLT.cpp file.