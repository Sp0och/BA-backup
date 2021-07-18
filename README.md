# Image And Descriptor Package

## Dependencies

- jsk_rviz_plugin

```bash
sudo apt install ros-noetic-jsk-rviz-plugins
```

## Running Code

Using the launch file both the executable and the rosbag can be ran simultaneously (after updating rosbag path).
For the rosbag file considered in this work (outdoor handheld dataset fromt shan et al. "Robust place reconition using visual lidars) visit:
https://drive.google.com/drive/folders/1G1kE8oYGKj7EMdjx7muGucXkt78cfKKU

### General Instructions

Plotting results:
"visualizer.py"
3 types of plotting:

- estimated numbers compard to GT and LOAM (pose and steps)
- errors regarding GT (pose and steps)
- Trajectory

Filename as well as parameters can be set for notation at the very top of the file.
Plot storage as well as legends and usage of special legends with error indication (only error comparison) can be toggled in the function calls.

Filtering:

- Points that get marked in the imagehandler as being too close to the sensor get filtered out in the get_3D_data function in the extractor classes.
- All other filtering instances are applied visibly and upfront
