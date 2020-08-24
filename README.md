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
```
SUBSYSTEM=="usb", ATTR{idProduct}=="0609", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
```
This is because 0609 is not in the default udev rules that get installed with the libopenni2. See [here](https://answers.ros.org/question/197318/openni2_launch-doesnt-work-with-carmine-109-connected-to-usb30/).

Note that for some system, the **/lib/udev/rules.d/40-libopenni2-0.rules** may not exist. Look for **/lib/udev/rules.d/60-libopenni2-0.rules** in this case.

## Alternative for gcc and g++ on Ubuntu
[Source link](https://askubuntu.com/questions/26498/how-to-choose-the-default-gcc-and-g-version)

To change the gcc and g++ version, first erase the current update-alternatives setup for gcc and g++:
```
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
```

Then install the gcc and g++ package. Here, we install the gcc-4.8 and g++-4.8:
```
sudo apt-get install gcc-4.8 g++-4.8
```

Symbolic link cc and c++ are installed by default. We need to install symbol links for gcc and g++, then link cc and c++ to gcc and g++, respectively. Say we have gcc-4.8 and gcc-6 in the computer
```
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 20

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 20

sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
sudo update-alternatives --set cc /usr/bin/gcc

sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++
```

The last step is to configure the default commands for gcc and g++:

```
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```

Finally, check the gcc version:
```
gcc -v
```
If you are working with ROS Kinetic, you should use gcc 5.x (e.g., gcc 5.5) for building. For reference, see [here](https://answers.ros.org/question/327497/compiling-ros-on-raspberry-pi-4-with-buster-problem-with-libboost158/).

## OpenCV Installation on Linux
1. Download the corresponding release zip file from the [official OpenCV website](https://opencv.org/releases/)
2. Unzip the zip file
'''
unzip opencv-x.x.xx.x.zip /destination_folder
'''
3. Follow the [official instartion](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) to build

### Installation problems with CUDA and solution
For **CMAake Error: Variables are set to NOTFOUND**, follow this [link](https://stackoverflow.com/questions/46584000/cmake-error-variables-are-set-to-notfound)

### Changing the link of the library
1. Check the the runtime library, for example

```
ldd demo
ldd demo | grep opencv
```
2. Then go to the folder where the library link to and make the original link as backup file, for example
```
mv libopencv_core.so.2.4 libopencv_core.so.2.4.bak
```
3. **Soft link** the so file to the target lib you want, for example
```
sudo ln -s <target so file> libopencv_core.so.2.4
```
target so file can be: libopencv_core.so.2.4.13

### NVCC compile adding library

To add the library which is pointing towards a specific path:

```
nvcc --gpu-architecture=sm_50 a.o b.o --library-path=<path> --library=foo
```

For example, in the [TSDF Fusion](https://github.com/jaydenwu17/tsdf-fusion) project, to compile

```
nvcc -std=c++11 -O3 -o demo demo.cu -I/usr/local/cuda/include -L$CUDA_LIB_DIR -lcudart -lcublas -lcurand -D_MWAITXINTRIN_H_INCLUDED --library-path=/home/hongtao/src_protected/opencv-2.4.13.6/build/lib  -lopencv_core -lopencv_highgui -lopencv_imgproc
```

## Set up zsh on Linux
### Step 1: Install and configure zsh
```
sudo apt install zsh
```
Change the default sheel of the root user to zsh:
```
chsh -s /usr/bin/zsh root
```
Check the current shell used:
```
echo $SHELL
```

### Step 2: Install and configure Oh-my-zsh
Install git:
```
sudo apt install wget git
```
Download the installer script:
```
wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh
```
Copy the template configuration file to home and apply the configuration:
```
cp ~/.oh-my-zsh/templates/zshrc.zsh-template ~/.zshrc
source ~/.zshrc
```

### [Optional] Step 3: Change default themes
```
cd ~/.oh-my-zsh/themes/
ls -a
```
```
vim ~/.zshrc
```
At *ZSH_THEME*, change it to say:
```
ZSH_THEME='risto'
```
And source the .zshrc.

More details can be found [here](https://www.howtoforge.com/tutorial/how-to-setup-zsh-and-oh-my-zsh-on-linux/)

### Configure the .zshrc for ROS
Add the following to the ~/.zshrc file
```
. /opt/ros/kinetic/setup.zsh
```

## Using ArUco tag in ROS
[Source link](https://blog.csdn.net/huanghaihui_123/article/details/88965426)
The ROS package to work with is the **aruco_ros**. It can be installed by building the [official git repository](https://github.com/pal-robotics/aruco_ros) or simply apt install:
```
sudo apt install ros-kinetic-aruco-ros
```

The aruco tag can be downloaded from [here](http://chev.me/arucogen/). But make sure in the Dictionary option, choose the **Orignal ArUco**! 

To use the package, we need to first remap the rosparam **camera_info** and **/image**. If you install with apt, then go to the launch file:
```
cd /opt/ros/kinetic/share/aruco_ros/launch
vim single.launch
```

Change the following to the target camera_info and image rostopic from your camera, for example
```
<remap from="/camera_info" to="/camera/rgb/camera_info">
<remap from="/image" to="/camera/rbg/image_rect">
```

Run the launch file
```
roslaunch arudo_ros single.launch markerId:=<marker id> markerSize:=<marker size>
```

To visualize the result
```
rosrun image_view image_view image:=/aruco_single/result
```

To check the pose with respect to the camera (marker pose in camear frame)
```
rostopic echo /aruco_single/pose
```
It is going to give the position in meter and the orientation in quaternion of the tag. 

### Axis orientation does not match issue
The axis printed out by the aruco tag when visualizing in the image_view does **not** match with the pose from the rostopic **/aruco_ros/pose**. Please be careful.

## UR5
### TCP position and orientation for UR5
We can pass tcp common to the UR5 to control its pose. The position is represented by [x, y, z]. The orientation is a bit tricky: it is represented by [rx, ry, rz]. The length of [rx, ry, rz] is the anlge to be rotated in radians, and the vector itself gives the axis about which to rotate.

### Using I/O to control the gripper ([afag EU-20 UR](https://www.afag.com/en/products/detail/universal-gripper-ug-20.html))
1. Connect the gripper to the gripper. Check the working voltage of the gripper. Usually it is either 12V or 24V.
2. In the UR interface, go to the I/O page. At the right low corner, set the voltage to the working voltage (the working voltage for afag EU-20 UR is 24V).
3. Click on the 0 in the Tool Output and see if the gripper is working.
4. To control the gripper from the computer, use the following
```
import urx
rob = urx.Robot("tcp address")
rob.send_program("set_tool_digital_out(%s, %s)" % (0, True))
rob.send_program("set_tool_digital_out(%s, %s)" % (0, False))
```
More details about the function **send_program** can be checked in [here](https://github.com/jaydenwu17/Engineering_Note/blob/master/official_ur5script.pdf). There are many available functions which is very handy to use.

## PrimeSense Camera

The x-axis of the camera frame is to the left of the camera when facing the camera lens. The y-axis is to the ground when facing the camera lens upright.

When not specified, enabling [depth_registration:=true], the depth image and the rgb image are registered by the default factory setting.

## Installing Caffe
Clone the repository
```
git clone https://github.com/BVLC/caffe.git
```
Make sure cuda, opencv are installed. To specify the opencv directory
```
export OpenCV_DIR="xxx/opencv-2.4.13.6/build"
```
Follow the instruction in https://blog.csdn.net/weixin_43915208/article/details/87885956 to build caffe.

Make sure to specify using CUDNN, OpenCV, and Python Version in the Makefile.config.

## Intel Realsense D435 setup guide
1. Install the **librealsense** library from https://github.com/IntelRealSense/librealsense. It is better to install from source. More details can be found here: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md.
2. Install the ROS wrapper **realsense-ros** with
  ```
  sudo apt-get install ros-$ROS_VER-realsense2-camera
  ```
  More details can be found from https://github.com/IntelRealSense/realsense-ros
  
## Set up UR5 robot with ur_modern_driver
The following works for Ubuntu 16.04 and ROS Kinetic. If you are using Ubuntu 18.04 and ROS Melodic, please use [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) instead. The tutorial to work with UR5 robot with ur_modern_driver can be found [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial). But this is very old documentation.
1. cd into the catkin_ws/src
```
git clone https://github.com/ros-industrial/ur_modern_driver.git
```
2. However, the master branch of this repo is deprecated and does not work for ROS kinetic. Thus, switch to the **kinetic-devel** branch. More details can be found [here](https://github.com/ros-industrial/ur_modern_driver/pull/120).
```
git checkout kinetic-devel
```
3. Then, catkin_make from catkin_ws/
4. To bring up the robot
```
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=$ROBOT_IP_ADDRESS
```

