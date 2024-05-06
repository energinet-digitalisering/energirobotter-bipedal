# Bipedal Description

Robot URFD description of bipedal system.

## Visualise URDF

With the `urdf_launch` ROS 2 package, the URDF file can be visualised and tested. Install dependency with:

```
rosdep install --from-paths src -y --ignore-src
```

And run with:
```
ros2 launch urdf_launch display.launch.py urdf_package:=bipedal_description urdf_package_path:=urdf/bipedal.urdf rviz_config:=src/reachy_bipedal/bipedal_description/config/bipedal_display.rviz
```

