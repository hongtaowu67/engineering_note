# Engineering_Note
Recording my failed and successful trial in engineering

## RGBD Camera with ROS
Use the [openni2](https://github.com/ros-drivers/openni2_camera) package. To __launch__ the rosnode ro run the camera

```
roslaunch openni2_launch openni2.launch
```

You can also add the _depth_registraition_

```
roslaunch openni2_launch openni2.launch depth_registration:=true
```

To check the RGB image

```
rosrun image_view image_view image:=/camera/rgb/image_raw
```

The RGB, Depth, and IR image can be checked from listing to rostopic.

The camera can be calibrated following this [link](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration).