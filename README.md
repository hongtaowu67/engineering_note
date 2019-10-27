# Engineering_Note
Recording my failure and success in Engineering...

## RGBD Camera with ROS
Use the [openni2](https://github.com/ros-drivers/openni2_camera) package. To __launch__ the rosnode ro run the camera

```
roslaunch openni2_launch openni2.launch
```

You can also add the **depth_registration** keyword.

```
roslaunch openni2_launch openni2.launch depth_registration:=true
```

To check the RGB image

```
rosrun image_view image_view image:=/camera/rgb/image_raw
```

The RGB, Depth, and IR image can be checked from listing to rostopic.

The camera can be calibrated following this [link](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration).

For calibrating the IR sensor, use a Post-it note to cover the speckle projector so that the pattern would not influence the detection of the checkerboard.
