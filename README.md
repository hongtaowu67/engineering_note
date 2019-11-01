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

To run with the Primesense 1.09 camera, need to add the following to **/lib/udev/rules.d/40-libopenni2-0.rules**
'''
SUBSYSTEM=="usb", ATTR{idProduct}=="0609", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
'''
This is because 0609 is not in the default udev rules that get installed with the libopenni2. See (link)[https://answers.ros.org/question/197318/openni2_launch-doesnt-work-with-carmine-109-connected-to-usb30/] here.

## Alternative for gcc and g++ on Ubuntu
[Source link]{https://askubuntu.com/questions/26498/how-to-choose-the-default-gcc-and-g-version}

To change the gcc and g++ version, first erase the current update-alternatives setup for gcc and g++:'''
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
'''

Then install the gcc and g++ package. Here, we install the gcc-4.8 and g++-4.8:
'''
sudo apt-get install gcc-4.8 g++-4.8
'''

Symbolic link cc and c++ are installed by default. We need to install symbol links for gcc and g++, then link cc and c++ to gcc and g++, respectively. Say we have gcc-4.8 and gcc-6 in the computer
'''
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 20

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 20

sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
sudo update-alternatives --set cc /usr/bin/gcc

sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++
'''

The last step is to configure the default commands for gcc and g++:

'''
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
'''

Finally, check the gcc version:
'''
gcc -v
'''